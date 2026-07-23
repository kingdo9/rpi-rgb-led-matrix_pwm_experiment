# SPWM register-profile catalogs

Demo 15 and `--led-spwm-register-config=N` load these catalogs at runtime so
the complete diagnostic profile sets do not have to be compiled into the core
library and demo objects.

Each `.profiles` file is UTF-8, tab-delimited, and versioned by its first line:

```text
RGBMATRIX_SPWM_PROFILES_V4<TAB>panel<TAB>rgb|fixed<TAB>profile-count
```

Every remaining line contains these fixed fields:

```text
name  source  scan-types  payload
```

RGB payloads use `register|R-words|G-words|B-words`; fixed payloads use repeating
`slot,R,G,B` groups separated by semicolons. Register words are four-digit
hexadecimal values without a `0x` prefix.

The selected panel's complete catalog is validated and retained in memory on
first use; restart the process to reload catalog edits. Set
`SPWM_PROFILE_DIR` to override the catalog directory.
