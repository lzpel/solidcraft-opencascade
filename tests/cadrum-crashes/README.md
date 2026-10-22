# Scripts that crash the OpenCASCADE backend

Each script reproduces a process crash inside OpenCASCADE (an access violation no `Result`
catches), found by `solidcraft-cli fuzz`; the previous (truck) backend ran them.
They are the reproductions attached to lzpel/cadrum#317 and the reason for the entries of
`tests/cadrum-skiplist.txt`. Run one:

    cargo run --release -p solidcraft-cli -- run tests/cadrum-crashes/seed-114.json

When cadrum no longer crashes on a script, its skip-list entry goes and the script stays as the
regression check.
