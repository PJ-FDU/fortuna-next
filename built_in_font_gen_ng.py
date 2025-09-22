#!/usr/bin/env python3
"""
Lightweight replacement for built_in_font_gen.py that fixes lv_font_conv parameter passing.

This script mirrors the original behavior but constructs argv cleanly and resolves
the bundled FontAwesome font path relative to the script directory so lv_font_conv
can find it even when invoked from the project root.

Usage is intentionally compatible with the old script.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Create fonts for LVGL including the built-in symbols.")
    parser.add_argument('-s', '--size', type=int, metavar='px', help='Size of the font in px')
    parser.add_argument('--bpp', type=int, metavar='1,2,4', nargs='?', help='Bit per pixel', choices=[1,2,3,4,8])
    parser.add_argument('-r', '--range', nargs='+', metavar='start-end', default=['0x20-0x7F,0xB0,0x2022'], help='Ranges and/or characters to include. Default is 0x20-7F (ASCII).')
    parser.add_argument('--symbols', nargs='+', metavar='sym', default=[''], help='Symbols to include. E.g. --symbols ÁÉŐ')
    parser.add_argument('--font', metavar='file', nargs='?', default='Montserrat-Medium.ttf', help='A TTF or WOFF file')
    parser.add_argument('-o', '--output', nargs='?', metavar='file', help='Output file name. E.g. my_font_20.c')
    parser.add_argument('--compressed', action='store_true', help='Compress the bitmaps')
    parser.add_argument('--subpx', action='store_true', help='3 times wider letters for sub pixel rendering')

    args = parser.parse_args(argv)

    # Prepare flags
    compr_flags = [] if args.compressed else ["--no-compress", "--no-prefilter"]
    lcd_flag = ["--lcd"] if args.subpx else []

    # Built-in icon codepoints (kept from original script)
    syms = (
        "61441,61448,61451,61452,61452,61453,61457,61459,61461,61465,61468,61473,61478,61479,61480,61502,"
        "61507,61512,61515,61516,61517,61521,61522,61523,61524,61543,61544,61550,61552,61553,61556,61559,"
        "61560,61561,61563,61587,61589,61636,61637,61639,61641,61664,61671,61674,61683,61724,61732,61787,"
        "61931,62016,62017,62018,62019,62020,62087,62099,62212,62189,62810,63426,63650"
    )

    # Script dir for locating bundled FontAwesome
    script_dir = os.path.dirname(os.path.abspath(__file__))
    fa_candidates = [
        os.path.join(script_dir, 'FontAwesome5-Solid+Brands+Regular.woff'),
        os.path.join(script_dir, 'FontAwesome.ttf'),
        'FontAwesome5-Solid+Brands+Regular.woff',
        'FontAwesome.ttf',
    ]
    fa_path = None
    for p in fa_candidates:
        if os.path.isfile(p):
            fa_path = os.path.abspath(p)
            break

    # Build command list for subprocess
    # Resolve lv_font_conv executable: prefer full path, handle .cmd and .js cases (Windows/npm)
    lv_exec = None
    node_exec = shutil.which('node')
    # Try direct lv_font_conv
    lv_path = shutil.which('lv_font_conv')
    if lv_path:
        # If it's a .js file, call with node
        if lv_path.lower().endswith('.js') and node_exec:
            lv_exec = [node_exec, lv_path]
        else:
            lv_exec = [lv_path]
    else:
        # try lv_font_conv.cmd shim
        lv_cmd = shutil.which('lv_font_conv.cmd')
        if lv_cmd:
            lv_exec = [lv_cmd]
        else:
            # fallback to npx if available
            npx_exec = shutil.which('npx')
            if npx_exec:
                lv_exec = [npx_exec, 'lv_font_conv']

    if not lv_exec:
        print("Execution failed: command not found. Make sure 'lv_font_conv' is on PATH or available via npx.", file=sys.stderr)
        return 3

    cmd = list(lv_exec)
    cmd.extend(lcd_flag)
    cmd.extend(compr_flags)

    if args.bpp is None:
        print("Error: --bpp is required (1,2,3,4 or 8).", file=sys.stderr)
        return 2
    cmd.extend(["--bpp", str(args.bpp)])

    if args.size is None:
        print("Error: --size is required.", file=sys.stderr)
        return 2
    cmd.extend(["--size", str(args.size)])

    # First font (user font)
    user_font = args.font
    # If user passed a relative path, allow it; but also prefer absolute path for reliability
    user_font_path = os.path.abspath(user_font) if os.path.exists(user_font) else user_font
    cmd.extend(["--font", user_font_path])

    # The original script accepted a single --range argument (with commas) or multiple.
    # Pass each provided -r entry separately so it belongs to the previously declared font.
    # args.range is a list; often it is a single comma-separated string by default.
    for r in args.range:
        # keep raw r; lv_font_conv supports comma-separated lists inside a single -r
        cmd.extend(["-r", r])

    # If symbols specified (non-empty), pass them for the first font
    if args.symbols and len(args.symbols) > 0 and args.symbols[0] != '':
        # join multiple symbol args into one string
        sym_str = ''.join(args.symbols)
        cmd.extend(["--symbols", sym_str])

    # Now append FontAwesome font (for built-in icons) if available
    if fa_path:
        cmd.extend(["--font", fa_path])
        # Pass the built-in icons list as a -r for the FontAwesome font
        cmd.extend(["-r", syms])
    else:
        print("Warning: FontAwesome font not found in script directory; built-in icons will be skipped.", file=sys.stderr)

    # Output format and file
    cmd.extend(["--format", "lvgl"])

    if args.output:
        cmd.extend(["-o", args.output])

    # force fast kern format as original did
    cmd.append("--force-fast-kern-format")

    # Print the assembled command for debugging (similar to original behavior)
    print("Running:")
    print(' '.join(cmd))

    try:
        completed = subprocess.run(cmd, check=False)
        return completed.returncode
    except Exception as e:
        print("Execution failed:", e, file=sys.stderr)
        return 4


if __name__ == '__main__':
    sys.exit(main())
