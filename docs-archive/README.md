# XY-series Modbus-RTU Documentation Archive

Offline copies of primary-source documentation for the XY6020L, XY7025, XY-SK60, XY-SK120, and XY-SK120X buck converter modules. The synthesis used to build this crate lives in `../DATASHEET.md`; this directory preserves the underlying sources in case those URLs rot.

All files were fetched on **2026-04-27**. Each text file begins with a short header noting its original URL.

## Index

### tinkering4fun/XY6020L-Modbus
- `tinkering4fun-README.md` — Repo README from <https://github.com/tinkering4fun/XY6020L-Modbus>. Brief guide to the XY6020L Modbus interface.
- `tinkering4fun-XY6020L-Modbus-Interface.pdf` — Original register-map PDF from `doc/XY6020L-Modbus-Interface.pdf` in the same repo. The most-cited primary-source register reference for XY6020L.

### Jens3382/xy6020l (Arduino library)
- `jens3382-README.md` — Repo README from <https://github.com/Jens3382/xy6020l>. Library overview and usage.
- `jens3382-xy6020l.h` — `src/xy6020l.h`: header file containing the `HREG_IDX_*` register-index constants used by the library.
- `jens3382-xy6020l.cpp` — `src/xy6020l.cpp`: implementation, including read/write code and timing details.

### csvke/XY-SK120-Modbus-RTU-TTL
- `csvke-README.md` — Main repo README from <https://github.com/csvke/XY-SK120-Modbus-RTU-TTL>. PlatformIO/ESP32 project for SK120/SK120X/SK60.
- `csvke-XY-SKxxx_Config.h` — `include/XY-SKxxx_Config.h`: pin/serial config header.
- `csvke-data-group-vset.md` — Short note on the VSET data-group parameter mnemonics (CU/CC/LVP/OUP/OCP/OPP/OAH/OPH/OHP/ORP/ERP/PON).
- `csvke-XY-SK120-Modbus_Address.pdf` — Official-looking SK120 Modbus register table PDF (from the repo's `documentation/`).
- `csvke-XY-SK60.pdf` — SK60 manual PDF (`20240731220204XY-SK60.pdf`).
- `csvke-XY-SK120X.pdf` — SK120X manual PDF.

### Forum threads / web manuals
- `allaboutcircuits-forum-thread.md` — Reverse-engineering thread <https://forum.allaboutcircuits.com/threads/exploring-programming-a-xy6020l-power-supply-via-modbus.197022/>. Includes the early register-discovery posts (g-radmac, Nov 2023) and links to derivative implementations.
- `manuals-plus-xy7025.md` — Seller-supplied XY7025 user manual scraped from <https://manuals.plus/ae/1005008036046439>. Covers specs, button operation, data groups, and protection ranges; does not include a Modbus register table.

## Sources not fetched

No XYSEMI/Sinilink official product page with Modbus register info was located in the brief search; community-maintained register tables (above) remain the de-facto primary references.
