#!/usr/bin/env python3
"""Mechanical BARR-C:2018 checks for the Subsystem 4 files.

This is a partial checker. It catches the rules that can be tested from the
text (line width, tabs, braces, forbidden keywords, raw integer types, short
names, switch default, if/else-if ending in else, equality with the constant
on the right, function length, single return, end-of-file comment). It does
NOT prove a file complies with BARR-C. Rules that need judgement (comment
quality, naming quality, magic numbers in general) stay a manual review.

Usage: barrc_check.py file.c [file.h ...]    exit status 1 if any FAIL.
"""
import re
import sys

KEYWORDS = ("continue", "goto", "register", "auto")
FUNC_DEF = re.compile(r"^[A-Za-z_][\w \*]*\b([A-Za-z_]\w*)\s*\(")


def strip(text):
    """Blank out comments and string/char literals, keep line structure."""
    out = []
    i, n = 0, len(text)
    while i < n:
        two = text[i:i + 2]
        if two == "/*":
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r"[^\n]", " ", text[i:j]))
            i = j
        elif two == "//":
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif text[i] in "\"'":
            q = text[i]
            j = i + 1
            while j < n and text[j] != q:
                j += 2 if text[j] == "\\" else 1
            j = min(j + 1, n)
            out.append(q + " " * (j - i - 2) + q if j - i >= 2 else q)
            i = j
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


def match_brace(code, start):
    depth = 0
    for k in range(start, len(code)):
        if code[k] == "{":
            depth += 1
        elif code[k] == "}":
            depth -= 1
            if depth == 0:
                return k
    return len(code) - 1


def check(path):
    raw = open(path, encoding="utf-8").read()
    lines = raw.split("\n")
    code = strip(raw)
    clines = code.split("\n")
    fails, warns = [], []

    def fail(n, msg):
        fails.append("%s:%d: %s" % (path, n, msg))

    def warn(n, msg):
        warns.append("%s:%d: %s" % (path, n, msg))

    for n, line in enumerate(lines, 1):
        if len(line) > 80:
            fail(n, "1.2 a line longer than 80 characters (%d)" % len(line))
        if "\t" in line:
            fail(n, "3.5 a tab character")
        if "\r" in line:
            fail(n, "3.6 a CR character")
    for n, line in enumerate(clines, 1):
        for kw in KEYWORDS:
            if re.search(r"\b%s\b" % kw, line):
                fail(n, "1.7 forbidden keyword '%s'" % kw)
        if re.search(r"\b(unsigned|short|long)\b", line):
            fail(n, "5.2 raw integer type (use uint32_t etc.)")
        elif re.search(r"\bint\b", line) and "snprintf" not in line:
            warn(n, "5.2 plain int: allowed only where an API demands it")
        if re.search(r"\b(exit|abort|setjmp|longjmp)\s*\(", line):
            fail(n, "8.5 b exit/abort/setjmp/longjmp")
        for m in re.finditer(
                r"(?:\bu?int\d+_t|\bbool|\bsize_t|\bchar)\s+\**\s*"
                r"([A-Za-z_]\w*)\s*(?:[=;,\[\)]|$)", line):
            name = m.group(1)
            if len(name) < 3 and not line.lstrip().startswith("#"):
                fail(n, "7.1 e variable name '%s' shorter than 3" % name)
        if re.search(r"[A-Za-z_\)\]]\s*==\s*(?:\d|[A-Z][A-Z0-9_]+\b)", line):
            fail(n, "8.6 a constant on the right of ==")
        if re.search(r"\bif\s*\([^;]*[^=!<>]=[^=][^;]*\)\s*$", line):
            warn(n, "8.2 c possible assignment inside if")
        sem = len(re.findall(r";", line))
        if sem > 1 and not re.search(r"\bfor\s*\(", line):
            fail(n, "3.3 a more than one statement on a line")

    # braces after control statements, Allman style
    for n, line in enumerate(clines, 1):
        s = line.strip()
        if re.match(r"(if|while|for|switch)\b", s) or re.match(r"else\b", s):
            if re.match(r"else\s+if\b", s):
                pass
            nxt = ""
            j = n
            body = s
            # join continuation lines of the condition
            depth = body.count("(") - body.count(")")
            while depth > 0 and j < len(clines):
                body += " " + clines[j].strip()
                depth = body.count("(") - body.count(")")
                j += 1
            rest_after = body
            if re.match(r"(?:\w+\s*)*\(", body) or body.startswith("else"):
                k = j
                while k < len(clines) and not clines[k].strip():
                    k += 1
                nxt = clines[k].strip() if k < len(clines) else ""
                if s.endswith("{") or body.rstrip().endswith("{"):
                    fail(n, "1.3 b left brace must be alone on its line")
                elif body.rstrip().endswith(";") and \
                        not re.match(r"do\b", s):
                    fail(n, "1.3 a braces required")
                elif nxt != "{" and not body.rstrip().endswith("{"):
                    if not (re.match(r"while\b", s) and
                            body.rstrip().endswith(";")):
                        fail(n, "1.3 a braces required")
            del rest_after

    # switch needs default; else-if chain needs else
    for m in re.finditer(r"\bswitch\b[^{]*\{", code):
        end = match_brace(code, m.end() - 1)
        n = code[:m.start()].count("\n") + 1
        if not re.search(r"\bdefault\s*:", code[m.end():end]):
            fail(n, "8.3 b switch without default")
    for m in re.finditer(r"\belse\s+if\b", code):
        j = code.find("{", m.end())
        k = match_brace(code, j)
        tail = code[k + 1:].lstrip()
        n = code[:m.start()].count("\n") + 1
        if not tail.startswith("else"):
            fail(n, "8.2 d else-if chain must end with else")

    # function length and single return
    pos = 0
    while True:
        j = code.find("\n{", pos)
        if j < 0:
            break
        k = match_brace(code, j + 1)
        if not code[:j].rstrip().endswith(")"):
            pos = k
            continue
        head = code[:j].rstrip().split("\n")
        name = "?"
        for h in reversed(head):
            m = re.search(r"\b([A-Za-z_]\w*)\s*\(", h)
            if m:
                name = m.group(1)
                break
        body = code[j:k]
        n0 = code[:j].count("\n") + 1
        length = body.count("\n")
        if length > 100:
            fail(n0, "6.2 a function %s is %d lines" % (name, length))
        if len(re.findall(r"\breturn\b", body)) > 1:
            warn(n0, "6.2 c function %s has several returns" % name)
        if name != name.lower() and not name.startswith("main"):
            fail(n0, "6.1 e uppercase in function name %s" % name)
        pos = k

    tail = raw.rstrip("\n").split("\n")[-1]
    if not tail.startswith("/* end of file"):
        fail(len(lines), "3.3 c missing end-of-file comment")
    return fails, warns


def main(argv):
    bad = 0
    for path in argv[1:]:
        fails, warns = check(path)
        for line in fails + warns:
            print(("FAIL " if line in fails else "warn ") + line)
        bad += len(fails)
        print("%-34s %d FAIL, %d warn" % (path, len(fails), len(warns)))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
