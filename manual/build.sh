#!/bin/sh
set -eu
manual_directory=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$manual_directory"
mkdir -p build pictures
compiler=${CXX:-c++}
"$compiler" -std=c++20 -Wall -Wextra -Wpedantic -Werror -I../include \
    pictures/generate_reference.cpp -o build/generate_reference
./build/generate_reference pictures
revision=$(git -C .. rev-parse --short=12 HEAD 2>/dev/null || printf '%s' 'unavailable')
printf '\\def\\ManualRevision{%s}\n' "$revision" > pictures/revision.tex
for pass in 1 2 3; do
    pdflatex -interaction=nonstopmode -halt-on-error -file-line-error \
        -output-directory=build main.tex > "build/latex-pass-$pass.log" 2>&1 || {
        tail -n 65 "build/latex-pass-$pass.log"
        exit 1
    }
done
cp build/main.pdf nurbspath-manual.pdf
printf '%s\n' 'Built manual/nurbspath-manual.pdf using local resources only.'
