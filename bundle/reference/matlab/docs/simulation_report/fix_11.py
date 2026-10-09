import re, sys

path = "sections/11_replication.tex"
with open(path, encoding="utf-8") as f:
    s = f.read()

# The file currently uses TWO backslashes for every LaTeX command
# (e.g. \\section instead of \section), except row breaks which also
# appear as \\ at end of line. Convert commands to single backslash
# while preserving row breaks.
def fix_line(ln):
    out = []
    i = 0
    n = len(ln)
    while i < n:
        if ln[i] == "\\" and i + 1 < n and ln[i + 1] == "\\":
            nxt = ln[i + 2] if i + 2 < n else None
            if nxt is None:          # row break at end of line -> keep
                out.append("\\\\")
                i += 2
                continue
            if nxt == "&":           # row break continuation -> keep
                out.append("\\\\")
                i += 2
                continue
            if nxt.isalpha():        # command name
                j = i + 2
                while j < n:
                    c = ln[j]
                    if c.isalpha():
                        j += 1
                        continue
                    if c == "\\" and j + 1 < n and ln[j + 1] in "_@!?~`\\":
                        j += 2
                        continue
                    break
                out.append("\\" + ln[i + 2:j])
                i = j
                continue
            else:                    # escaped char: \\_ \\, \\%, \\", \\}
                out.append("\\" + nxt)
                i += 2
                continue
        else:
            out.append(ln[i])
            i += 1
    return "".join(out)

fixed = "\n".join(fix_line(ln) for ln in s.split("\n"))

with open(path, "w", encoding="utf-8") as f:
    f.write(fixed)

# verify
with open(path, encoding="utf-8") as f:
    now = f.read()
checks = {
    "\\section": now.count("\\section"),
    "\\subsection": now.count("\\subsection"),
    "\\subsubsection": now.count("\\subsubsection"),
    "\\label": now.count("\\label"),
    "\\ref": now.count("\\ref"),
    "\\input": now.count("\\input"),
    "\\code": now.count("\\code"),
    "\\caption": now.count("\\caption"),
    "\\label{tab:cross}]": now.count("\\label{tab:cross}]"),
    "\\S": now.count("\\S"),
    "\\_": now.count("\\_"),
    "\\,": now.count("\\,"),
    "\\%": now.count("\\%"),
    "95\\%": now.count("95\\%"),
    "Table\\,": now.count("Table\\,"),
    "Section~\\S": now.count("Section~\\S"),
    "60\\degree": now.count("60\\degree"),
    "\\begin": now.count("\\begin"),
    "\\end": now.count("\\end"),
}
for k, v in checks.items():
    print(f"{k}: {v}")
bad = re.findall(r"\\\\(?![a-zA-Z_@!?~`\\}]|,%|$)", now)
print("double-backslash pairs left (bad):", len(bad))
