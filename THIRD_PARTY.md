# Third-Party Licenses

Lumice bundles the following third-party assets. The licenses below cover the
embedded / vendored copies; please refer to each upstream project for the
authoritative license text and complete attribution.

## Font Awesome 6 Free (Solid)

- Source: https://fontawesome.com/
- Upstream: https://github.com/FortAwesome/Font-Awesome (tag `6.7.2`)
- File embedded: `webfonts/fa-solid-900.ttf`
- License: SIL Open Font License 1.1 (`OFL-1.1`)

The TTF is fetched at configure time via CPM and compiled into the GUI binary
by `scripts/embed_binary.py` (see `src/gui/CMakeLists.txt`).

## IconFontCppHeaders

- Source: https://github.com/juliettef/IconFontCppHeaders
- Pinned to commit `3ee7f3d295ae773c0046db8d7b89b886eb2526de`
- License: Zlib

Provides the `ICON_FA_*` macro definitions used to address glyphs in the
Font Awesome Solid font. Header-only, consumed by `src/gui/*.cpp`.
