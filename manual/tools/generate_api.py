#!/usr/bin/env python3
"""Refresh the manual's public API catalogue using the installed Doxygen.

No packages are installed and no CMake files are read or changed. Generated
LaTeX is retained so an ordinary manual build does not require Doxygen.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET


MANUAL = Path(__file__).resolve().parents[1]
ROOT = MANUAL.parent
BUILD = MANUAL / "build" / "api"


def text(element: ET.Element | None) -> str:
    if element is None:
        return ""
    return re.sub(r"\s+", " ", "".join(element.itertext())).strip()


def escape(value: str) -> str:
    replacements = {
        "\\": r"\textbackslash{}", "&": r"\&", "%": r"\%",
        "$": r"\$", "#": r"\#", "_": r"\_", "{": r"\{",
        "}": r"\}", "~": r"\textasciitilde{}", "^": r"\textasciicircum{}",
    }
    return "".join(replacements.get(character, character) for character in value)


def signature(member: ET.Element) -> str:
    name = text(member.find("name"))
    kind = member.get("kind")
    result = text(member.find("type"))
    prefix = ""
    template = member.find("templateparamlist")
    if template is not None:
        parameters = []
        for param in template.findall("param"):
            declaration = " ".join(filter(None, [text(param.find("type")),
                                                text(param.find("declname"))]))
            default = text(param.find("defval"))
            parameters.append(declaration + (" = " + default if default else ""))
        prefix = "template <" + ", ".join(parameters) + ">\n"
    if kind == "function":
        if member.get("static") == "yes":
            prefix += "static "
        return prefix + (result + " " if result else "") + name + text(member.find("argsstring")) + ";"
    if kind == "typedef":
        return prefix + "using " + name + " = " + result + ";"
    if kind == "enum":
        values = [text(value.find("name")) for value in member.findall("enumvalue")]
        return "enum class " + name + " { " + ", ".join(values) + " };"
    initializer = text(member.find("initializer"))
    return (result + " " if result else "") + name + (" " + initializer if initializer else "") + ";"


def public_members(compound: ET.Element) -> list[ET.Element]:
    return [member for section in compound.findall("sectiondef")
            for member in section.findall("memberdef")
            if member.get("prot", "public") == "public"
            and member.get("kind") in {"function", "variable", "typedef", "enum"}]


def render_members(members: list[ET.Element], output: list[str]) -> None:
    for member in members:
        brief = text(member.find("briefdescription"))
        output.extend([r"\par\noindent\begin{minipage}{\linewidth}",
                       r"\begin{lstlisting}[style=apireference]",
                       signature(member), r"\end{lstlisting}"])
        if brief:
            output.append(r"{\footnotesize " + escape(brief) + r"\par}")
        for description in member.findall("detaileddescription/para/simplesect"):
            if description.get("kind") == "return":
                output.append(r"{\footnotesize\textit{Returns:} " +
                              escape(text(description)) + r"\par}")
        output.append(r"\end{minipage}\par\vspace{1pt}")


def main() -> None:
    if not shutil.which("doxygen"):
        raise SystemExit("Doxygen is not installed; the checked-in catalogue remains usable.")
    BUILD.mkdir(parents=True, exist_ok=True)
    config = BUILD / "Doxyfile"
    config.write_text(f'''PROJECT_NAME = nurbspath_manual
INPUT = "{ROOT / 'include' / 'nurbspath'}"
OUTPUT_DIRECTORY = "{BUILD}"
RECURSIVE = YES
GENERATE_HTML = NO
GENERATE_LATEX = NO
GENERATE_XML = YES
XML_PROGRAMLISTING = NO
EXTRACT_ALL = NO
EXTRACT_PRIVATE = NO
EXTRACT_STATIC = NO
WARN_IF_UNDOCUMENTED = YES
WARN_IF_DOC_ERROR = YES
WARN_AS_ERROR = YES
QUIET = YES
''')
    for attempt in range(3):
        result = subprocess.run(["doxygen", str(config)], cwd=MANUAL,
                                capture_output=True, text=True)
        (BUILD / "doxygen.log").write_text(result.stdout + result.stderr)
        if result.returncode == 0:
            break
        # The locally installed 1.18 lexer can fail nondeterministically. Retry
        # only a process signal, never suppress a documentation error.
        if result.returncode >= 0 or attempt == 2:
            raise SystemExit(result.stdout + result.stderr or
                             f"Doxygen terminated with signal {-result.returncode}")
    parsed = []
    hidden_names = set()
    for path in sorted((BUILD / "xml").glob("*.xml")):
        document = ET.parse(path)
        compound = document.find("compounddef")
        if compound is None:
            continue
        parsed.append(compound)
        for nested in compound.findall("innerclass"):
            if nested.get("prot", "public") != "public":
                hidden_names.add(text(nested))
    compounds = []
    for compound in parsed:
        name = text(compound.find("compoundname"))
        if (compound.get("kind") in {"class", "struct"}
                and name.startswith("nurbspath::") and "::detail::" not in name
                and compound.get("prot", "public") == "public"
                and not any(name == hidden or name.startswith(hidden + "::")
                            for hidden in hidden_names)):
            compounds.append((name, compound))
    namespace = ET.parse(BUILD / "xml" / "namespacenurbspath.xml").find("compounddef")
    if namespace is None:
        raise SystemExit("Doxygen did not emit the public nurbspath namespace.")
    free = public_members(namespace)
    by_header: dict[str, list[ET.Element]] = {}
    for member in free:
        location = member.find("location")
        header = Path(location.get("file", "unknown.hpp")).name if location is not None else "unknown.hpp"
        by_header.setdefault(header, []).append(member)
    inventory = {"classes": {}, "namespace_headers": {}, "function_count": 0,
                 "member_count": 0}
    lines = ["% Generated from public Doxygen XML by tools/generate_api.py.",
             r"\begingroup\setlength{\parskip}{2pt}"]
    for name, compound in sorted(compounds):
        members = public_members(compound)
        inventory["classes"][name] = [text(member.find("name")) for member in members]
        inventory["member_count"] += len(members)
        inventory["function_count"] += sum(member.get("kind") == "function" for member in members)
        lines.append(r"\Needspace{9\baselineskip}")
        lines.append(r"\section{" + escape(name) + "}")
        brief = text(compound.find("briefdescription"))
        if brief:
            lines.append(escape(brief))
        render_members(members, lines)
    lines.append(r"\section{Namespace functions, aliases, and result enums}")
    for header, members in sorted(by_header.items()):
        inventory["namespace_headers"][header] = [text(member.find("name")) for member in members]
        inventory["member_count"] += len(members)
        inventory["function_count"] += sum(member.get("kind") == "function" for member in members)
        lines.append(r"\Needspace{9\baselineskip}")
        lines.append(r"\subsection{" + escape(header) + "}")
        render_members(members, lines)
    lines.append(r"\endgroup")
    (MANUAL / "sections" / "reference_api.tex").write_text("\n".join(lines) + "\n")
    (MANUAL / "pictures" / "api_inventory.json").write_text(json.dumps(inventory, indent=2) + "\n")
    print(f"API catalogue: {len(compounds)} public types; "
          f"{inventory['function_count']} callable overloads; "
          f"{inventory['member_count']} total entries.")


if __name__ == "__main__":
    main()
