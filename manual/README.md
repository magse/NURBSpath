# Standalone illustrated manual

Read the [PDF manual](nurbspath-manual.pdf) for illustrated entity construction,
algorithm examples, and the public API catalogue. See the
[project overview](../README.md), [installation guide](../INSTALL.md), and
[tagged data format](../DATA.md) for the accompanying project documentation.

Build from the repository root:

```sh
sh manual/build.sh
```

The result is `manual/nurbspath-manual.pdf`. The script uses the existing C++20
compiler and local `pdflatex`; it does not install or download anything and
does not use CMake. Compiler/TeX intermediates are confined to `manual/build/`.

The [main LaTeX file](main.tex) includes separate files in `sections/` with
shared styling in [preamble.tex](preamble.tex).
[pictures/generate_reference.cpp](pictures/generate_reference.cpp) uses the
library to regenerate and check
reference coordinates in `pictures/`. TikZ and PGFPlots render those coordinates
inside the PDF. Displayed numerical values use at most four decimal places in
scientific notation; computations are checked before rounding.

The installed TeX distribution must already include the packages listed in
`preamble.tex`. The current environment provides them in TeX Live 2026. No
shell escape, external image service, or TikZ externalisation is used.

Optional `intro.tex` is included after the cover and before the contents;
optional `external.tex` is included after the appendices. Supply them as LaTeX
fragments without a document preamble. They may be absent.

The retained API catalogue was extracted from the public headers. To refresh
it with an already installed Python and Doxygen:

```sh
cd manual
python3 tools/generate_api.py
sh build.sh
```

`pictures/api_inventory.json` records the extracted coverage. The normal build
uses `sections/reference_api.tex` and therefore does not require Doxygen.

The manual builds independently of CMake. Its diagrams explain the public API;
the complete library test suite remains the numerical regression authority.
