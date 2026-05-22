"""Parser for RomRaider log_defs.xml.

Loads the XML into typed dataclasses and resolves the `include` chain so a
caller can ask "what parameters apply to ROM ID X?" and get a flat dict back.

Schema (informal - RomRaider's own logger.dtd does not match this file):

    <ecus>
      <ecu_tools>
        <convert_factor type="..." name="..." metric="..." expr="..."/> ...
      </ecu_tools>
      <logprotocols>
        <logprotocol type="SSM" default="ssmbase">
          <ecu id="base" name="..." type="ssmbase">          # base sets
            <parameter id="Engine Speed" offset="#000E"
                       storagetype="uint16" bit="1" byte="1"
                       expr="[value]/4" metric="RPM" desc="..."/>
            <parameter id="Advance Multiplier" offset="#20124" ...>
              <alt id="..." offset="#20118"/>                # variants
              ...
            </parameter>
            ...
          </ecu>
          <ecu id="base" name="..." type="ssmbase16"  include="ssmbase">  ...</ecu>
          <ecu id="base" name="..." type="ssmbase32"  include="ssmbase">  ...</ecu>
          <ecu id="1644500305" name="..." type="AE800"
               include="ssmbase16"/>                          # leaf ECUs
          ...
        </logprotocol>
      </logprotocols>
    </ecus>

Notes:
- The `id` on a leaf ECU is the 5-byte SSM2 ROM ID returned by the 0xAA init,
  hex-encoded (10 chars). Some entries are shorter (2-4 bytes) - these are
  prefix matches for older ECUs whose init returned a shorter ID.
- `bit`/`byte` are 1-based indexes into the capability-flag bytes that come
  back with the 0xAA init response. A parameter is supported iff that bit is
  set. Parameters without bit/byte (the `<alt>`-style ones) are gated by
  ROM-specific knowledge instead.
- `offset` is "#" + hex. SSM2 addresses on 16/32-bit ECUs can exceed 24 bits.
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterator


def _parse_offset(s: str | None) -> int | None:
    if s is None:
        return None
    s = s.strip()
    if s.startswith("#"):
        s = s[1:]
    return int(s, 16)


def _parse_int(s: str | None) -> int | None:
    return int(s) if s is not None else None


@dataclass(frozen=True)
class ConvertFactor:
    """A unit-conversion helper from <ecu_tools>."""
    type: str
    name: str
    metric: str
    expr: str


@dataclass(frozen=True)
class AltOffset:
    """An <alt> child of a <parameter> - alternative offset for an ECU variant."""
    id: str
    offset: int
    storagetype: str | None = None


@dataclass
class Parameter:
    """One <parameter> entry. `name` is the human-readable id attribute."""
    name: str
    offset: int | None              # None when only alts apply
    storagetype: str | None
    decimals: int | None
    bit: int | None                 # 1-based capability-flag bit, or None
    byte: int | None                # 1-based capability-flag byte, or None
    expr: str | None                # raw conversion expression with [value]
    metric: str | None              # unit string (display)
    desc: str                       # may use '|' as line break
    factor_type: str | None         # references ConvertFactor.type, if any
    alts: tuple[AltOffset, ...] = field(default_factory=tuple)


@dataclass
class Ecu:
    """A single <ecu> entry. For base sets, `id` == 'base'."""
    id: str                         # ROM ID hex string, or 'base'
    name: str
    type: str                       # used as the include() target
    include: str | None             # parent type to inherit parameters from
    parameters: dict[str, Parameter] = field(default_factory=dict)


@dataclass
class LogDefs:
    convert_factors: dict[str, ConvertFactor]
    ecus_by_type: dict[str, Ecu]    # ssmbase, ssmbase16, AE800, ...
    ecus_by_rom: dict[str, Ecu]     # ROM ID hex -> Ecu (skips 'base' entries)
    source_path: Path | None = None

    @classmethod
    def load(cls, path: str | Path) -> "LogDefs":
        path = Path(path)
        tree = ET.parse(path)
        root = tree.getroot()
        if root.tag != "ecus":
            raise ValueError(f"unexpected root element <{root.tag}>; expected <ecus>")

        convert_factors: dict[str, ConvertFactor] = {}
        for cf in root.findall("./ecu_tools/convert_factor"):
            t = cf.get("type")
            if t is None:
                continue
            convert_factors[t] = ConvertFactor(
                type=t,
                name=cf.get("name", ""),
                metric=cf.get("metric", ""),
                expr=cf.get("expr", ""),
            )

        ecus_by_type: dict[str, Ecu] = {}
        ecus_by_rom: dict[str, Ecu] = {}
        for proto in root.findall("./logprotocols/logprotocol"):
            if proto.get("type") != "SSM":
                continue
            for ecu_el in proto.findall("./ecu"):
                ecu = _parse_ecu(ecu_el)
                if ecu.type in ecus_by_type and ecu.id == "base":
                    # base ECUs have type strings that should be unique
                    raise ValueError(f"duplicate base ECU type: {ecu.type!r}")
                if ecu.type and ecu.id == "base":
                    ecus_by_type[ecu.type] = ecu
                elif ecu.id:
                    ecus_by_rom[ecu.id.upper()] = ecu
                    # Also expose under their type for include() lookups,
                    # but only if not already populated by a base set.
                    if ecu.type and ecu.type not in ecus_by_type:
                        ecus_by_type[ecu.type] = ecu

        return cls(
            convert_factors=convert_factors,
            ecus_by_type=ecus_by_type,
            ecus_by_rom=ecus_by_rom,
            source_path=path,
        )

    def resolve(self, key: str) -> dict[str, Parameter]:
        """Return all parameters applicable to an ECU, after walking includes.

        `key` may be a ROM ID hex string (case-insensitive) or a type name
        like "ssmbase16" or "AE800". Later includes override earlier on
        parameter-name collision.
        """
        ecu = self.find_ecu(key)
        if ecu is None:
            raise KeyError(f"no ECU matching {key!r}")
        merged: dict[str, Parameter] = {}
        for ancestor in self._include_chain(ecu):
            merged.update(ancestor.parameters)
        return merged

    def find_ecu(self, key: str) -> Ecu | None:
        """Look up by ROM ID (exact, then prefix), then by type name."""
        k = key.upper()
        if k in self.ecus_by_rom:
            return self.ecus_by_rom[k]
        # Prefix match - some entries are truncated ROM IDs (older ECUs).
        for rom, ecu in self.ecus_by_rom.items():
            if k.startswith(rom) or rom.startswith(k):
                return ecu
        return self.ecus_by_type.get(key)

    def _include_chain(self, ecu: Ecu) -> Iterator[Ecu]:
        """Yield ancestors first, then `ecu` itself."""
        if ecu.include:
            parent = self.ecus_by_type.get(ecu.include)
            if parent is None:
                raise ValueError(
                    f"ECU {ecu.id!r} (type={ecu.type!r}) includes unknown {ecu.include!r}"
                )
            yield from self._include_chain(parent)
        yield ecu


def _parse_ecu(el: ET.Element) -> Ecu:
    ecu = Ecu(
        id=el.get("id", ""),
        name=el.get("name", ""),
        type=el.get("type", ""),
        include=el.get("include"),
    )
    for pel in el.findall("./parameter"):
        p = _parse_parameter(pel)
        ecu.parameters[p.name] = p
    return ecu


def _parse_parameter(el: ET.Element) -> Parameter:
    alts = tuple(
        AltOffset(
            id=a.get("id", ""),
            offset=_parse_offset(a.get("offset")) or 0,
            storagetype=a.get("storagetype"),
        )
        for a in el.findall("./alt")
    )
    return Parameter(
        name=el.get("id", ""),
        offset=_parse_offset(el.get("offset")),
        storagetype=el.get("storagetype"),
        decimals=_parse_int(el.get("decimals")),
        bit=_parse_int(el.get("bit")),
        byte=_parse_int(el.get("byte")),
        expr=el.get("expr"),
        metric=el.get("metric"),
        desc=el.get("desc", ""),
        factor_type=el.get("type"),
        alts=alts,
    )


_STORAGE_MAP = {
    "uint8":  "Uint8",
    "uint16": "Uint16",
    "uint32": "Uint32",
    "int8":   "Int8",
    "int16":  "Int16",
    "int32":  "Int32",
    "float":  "Float",
}


def _cpp_string(s: str) -> str:
    """Escape a Python string as a C++ string literal.

    Converts RomRaider's '|' line-break convention to literal newlines.
    """
    if not s:
        return '""'
    s = s.replace("|", "\n")
    out = []
    for ch in s:
        cp = ord(ch)
        if ch == "\\":
            out.append("\\\\")
        elif ch == '"':
            out.append('\\"')
        elif ch == "\n":
            out.append("\\n")
        elif ch == "\t":
            out.append("\\t")
        elif ch == "\r":
            out.append("\\r")
        elif 32 <= cp <= 126:
            out.append(ch)
        else:
            out.append(f"\\x{cp:02x}")
    return '"' + "".join(out) + '"'


def _emit_cpp(defs: LogDefs, base_type: str, out_path: Path) -> int:
    """Emit a self-contained C++17 header with the resolved parameter table.

    Returns the number of parameters written.
    """
    params = list(defs.resolve(base_type).values())
    # Sort: flag-gated first by (byte, bit), then any non-gated by name.
    params.sort(key=lambda p: (
        0 if (p.byte and p.bit) else 1,
        p.byte or 0, p.bit or 0, p.name,
    ))

    src_name = defs.source_path.name if defs.source_path else "<unknown>"
    src_hash = ""
    if defs.source_path and defs.source_path.exists():
        src_hash = hashlib.sha256(defs.source_path.read_bytes()).hexdigest()[:16]
    timestamp = datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")

    lines: list[str] = []
    w = lines.append

    w(f"// {out_path.name} - auto-generated by gen_params.py")
    w("// DO NOT EDIT - regenerate from log_defs.xml")
    w("//")
    w(f"// Source XML  : {src_name}" + (f"  (sha256:{src_hash}...)" if src_hash else ""))
    w(f"// Base type   : {base_type}")
    w(f"// Parameters  : {len(params)}")
    w(f"// Generated   : {timestamp}")
    w("//")
    w("// This file is a derivative work of RomRaider's log_defs.xml,")
    w("// distributed under GPL-2.0+. See LICENSE.")
    w("")
    w("#pragma once")
    w("")
    w("#include <array>")
    w("#include <cstdint>")
    w("#include <string_view>")
    w("")
    w("namespace libssm2 {")
    w("")
    w("enum class StorageType : std::uint8_t {")
    w("    Uint8, Uint16, Uint32,")
    w("    Int8,  Int16,  Int32,")
    w("    Float,")
    w("    Unknown,")
    w("};")
    w("")
    w("struct CapFlag {")
    w("    std::uint8_t byte;  // 1-based index into the 96 cap-flag bytes; 0 == not gated")
    w("    std::uint8_t bit;   // 1-based bit within that byte (1..8);     0 == not gated")
    w("")
    w("    constexpr bool gated() const noexcept { return byte != 0 && bit != 0; }")
    w("};")
    w("")
    w("struct SsmParameter {")
    w("    std::string_view name;       // human-readable identifier")
    w("    std::uint32_t    offset;     // SSM2 read address (0 if only alts apply)")
    w("    CapFlag          cap;        // capability-flag gate from 0xAA init")
    w("    StorageType      storage;    // wire encoding")
    w("    std::uint8_t     decimals;   // display precision hint")
    w("    std::string_view metric;     // unit string")
    w("    std::string_view expr;       // conversion expression, uses [value]")
    w("    std::string_view factor;     // optional ecu_tools convert_factor key")
    w("    std::string_view desc;       // long description ('|' converted to newlines)")
    w("};")
    w("")
    w(f"inline constexpr std::array<SsmParameter, {len(params)}> kSsmBaseTable {{{{")

    for p in params:
        name    = _cpp_string(p.name)
        offset  = f"0x{p.offset:05X}" if p.offset is not None else "0x00000"
        cap     = f"{{ {p.byte or 0:>2}, {p.bit or 0:>2} }}"
        storage = f"StorageType::{_STORAGE_MAP.get(p.storagetype or '', 'Unknown')}"
        decim   = f"{p.decimals or 0}"
        metric  = _cpp_string(p.metric or "")
        expr    = _cpp_string(p.expr or "")
        factor  = _cpp_string(p.factor_type or "")
        desc    = _cpp_string(p.desc or "")

        # Compact short fields on one line, descriptions on a continuation line.
        w(f"    {{ {name}, {offset}, {cap}, {storage}, {decim}, {metric}, {expr}, {factor},")
        w(f"      {desc} }},")

    w("}};")
    w("")
    w("}  // namespace libssm2")
    w("")

    out_path.write_text("\n".join(lines), encoding="utf-8")
    return len(params)


def _format_desc(desc: str, width: int = 70, indent: str = "    ") -> str:
    lines = []
    for chunk in desc.split("|"):
        chunk = chunk.strip()
        if chunk:
            lines.append(indent + chunk)
    return "\n".join(lines)


def _cli(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description="Inspect a RomRaider log_defs.xml file.")
    p.add_argument("path", nargs="?", default="log_defs.xml", help="path to log_defs.xml")
    p.add_argument("--list-ecus", action="store_true", help="list all ECU entries")
    p.add_argument("--ecu", metavar="ID_OR_TYPE", help="resolve and print params for one ECU")
    p.add_argument("--search", metavar="TEXT", help="search parameter names (in ssmbase + ssmbase16 + ssmbase32)")
    p.add_argument("--full", action="store_true", help="include descriptions in --ecu output")
    p.add_argument("--cap-flags", metavar="HEX",
                   help="96-byte capability-flag string from an SSM2 0xAA init response; "
                        "lists which ssmbase parameters this ECU advertises support for")
    p.add_argument("--emit-cpp", action="store_true",
                   help="generate a self-contained C++17 header with the resolved parameter table")
    p.add_argument("--out", metavar="PATH", default="SsmBaseTable.h",
                   help="output path for --emit-cpp (default: SsmBaseTable.h)")
    p.add_argument("--base", metavar="TYPE", default="ssmbase",
                   help="which base set to emit (ssmbase, ssmbase16, ssmbase32; default ssmbase)")
    args = p.parse_args(argv)

    defs = LogDefs.load(args.path)
    print(f"loaded {args.path}: {len(defs.ecus_by_rom)} ROM IDs, "
          f"{len(defs.ecus_by_type)} types, {len(defs.convert_factors)} convert factors")

    if args.list_ecus:
        for rom in sorted(defs.ecus_by_rom):
            ecu = defs.ecus_by_rom[rom]
            print(f"  {rom:<12} {ecu.type:<14} {ecu.name}")
        return 0

    if args.ecu:
        params = defs.resolve(args.ecu)
        print(f"\n{len(params)} parameters for {args.ecu!r}:\n")
        for name, prm in sorted(params.items()):
            off = f"0x{prm.offset:X}" if prm.offset is not None else "-"
            bb = f"flag[{prm.byte}.{prm.bit}]" if prm.bit and prm.byte else ""
            print(f"  {name:<40}  {off:<10}  {prm.storagetype or '':<7}  {prm.metric or '':<8}  {bb}")
            if args.full and prm.desc:
                print(_format_desc(prm.desc))
            if prm.alts:
                for a in prm.alts:
                    print(f"      alt: 0x{a.offset:X} ({a.storagetype or '-'})")
        return 0

    if args.emit_cpp:
        out_path = Path(args.out)
        n = _emit_cpp(defs, args.base, out_path)
        print(f"wrote {n} parameters from {args.base!r} to {out_path}")
        return 0

    if args.cap_flags:
        hex_str = re.sub(r"[^0-9a-fA-F]", "", args.cap_flags)
        if len(hex_str) % 2:
            print("error: hex string must have an even number of nibbles", file=sys.stderr)
            return 2
        flags = bytes.fromhex(hex_str)
        if len(flags) != 96:
            print(f"error: expected 96 cap-flag bytes from SSM2 init, got {len(flags)}",
                  file=sys.stderr)
            return 2
        base = defs.ecus_by_type.get("ssmbase")
        if base is None:
            print("error: ssmbase not found in defs", file=sys.stderr)
            return 2
        supported = []
        unknown_byte = 0
        for prm in base.parameters.values():
            if prm.byte is None or prm.bit is None:
                continue
            byte_idx = prm.byte - 1   # 1-based to 0-based
            bit_mask = 1 << (prm.bit - 1)
            if byte_idx >= len(flags):
                unknown_byte = max(unknown_byte, byte_idx + 1)
                continue
            if flags[byte_idx] & bit_mask:
                supported.append(prm)
        supported.sort(key=lambda p: (p.byte, p.bit))
        print(f"\n{len(supported)} supported parameters (out of "
              f"{sum(1 for p in base.parameters.values() if p.byte and p.bit)} flag-gated in ssmbase):\n")
        for prm in supported:
            off = f"0x{prm.offset:X}" if prm.offset is not None else "-"
            bb = f"[byte {prm.byte:>2}.bit {prm.bit}]"
            print(f"  {bb}  {prm.name:<40}  {off:<8}  {prm.storagetype or '':<7}  {prm.metric or ''}")
        if unknown_byte:
            print(f"\nnote: flag stream too short to evaluate byte index {unknown_byte}",
                  file=sys.stderr)
        return 0

    if args.search:
        needle = args.search.lower()
        seen: set[str] = set()
        for base_type in ("ssmbase", "ssmbase16", "ssmbase32"):
            base = defs.ecus_by_type.get(base_type)
            if base is None:
                continue
            for name, prm in base.parameters.items():
                if needle in name.lower() and name not in seen:
                    seen.add(name)
                    off = f"0x{prm.offset:X}" if prm.offset is not None else "-"
                    print(f"  [{base_type}] {name:<40} {off:<10} {prm.metric or ''}")
        return 0

    p.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(_cli(sys.argv[1:]))
