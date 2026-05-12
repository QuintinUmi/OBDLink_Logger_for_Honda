from pathlib import Path

base = Path(r".\opendbc\opendbc\dbc\generator\honda")

files = [
    "_honda_common.dbc",
    "_gearbox_common.dbc",
    "_steering_sensors_b.dbc",
    "_nidec_common.dbc",
]

out = Path(r".\honda_mini_clean.dbc")

header = """VERSION ""

NS_ :
    NS_DESC_
    CM_
    BA_DEF_
    BA_
    VAL_
    CAT_DEF_
    CAT_
    FILTER
    BA_DEF_DEF_
    EV_DATA_
    ENVVAR_DATA_
    SGTYPE_
    SGTYPE_VAL_
    BA_DEF_SGTYPE_
    BA_SGTYPE_
    SIG_TYPE_REF_
    VAL_TABLE_
    SIG_GROUP_
    SIG_VALTYPE_
    SIGTYPE_VALTYPE_
    BO_TX_BU_
    BA_DEF_REL_
    BA_REL_
    BA_DEF_DEF_REL_
    BU_SG_REL_
    BU_EV_REL_
    BU_BO_REL_
    SG_MUL_VAL_

BS_:

BU_: ADAS RADAR NEO XXX EON EPS VSA PCM BDY EBCM INTERCEPTOR

"""

body = []
seen_ids = set()

for fn in files:
    path = base / fn
    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()

    i = 0
    while i < len(lines):
        line = lines[i].rstrip()

        if line.startswith("BO_ "):
            parts = line.split()
            frame_id = int(parts[1])

            block = [line]
            i += 1

            while i < len(lines) and not lines[i].startswith("BO_ "):
                l = lines[i].rstrip()

                # 只保留信号行，去掉 CM_ / VAL_ / IMPORT / header 等
                if l.strip().startswith("SG_ "):
                    block.append(l)

                i += 1

            if frame_id not in seen_ids:
                body.extend(block)
                body.append("")
                seen_ids.add(frame_id)

            continue

        i += 1

out.write_text(header + "\n".join(body), encoding="utf-8")

print(f"Wrote {out}")
print(f"Messages: {len(seen_ids)}")
for x in sorted(seen_ids):
    print(f"  {x:4d} 0x{x:X}")