#!/usr/bin/env python3
"""Inline all sources into one self-contained HTML page: dist/index.html"""
import pathlib, sys
SRC = pathlib.Path(__file__).parent / 'src'
OUT = pathlib.Path(__file__).parent / 'dist' / 'index.html'
read = lambda n: (SRC / n).read_text(encoding='utf-8')
js = '\n'.join(read(n) for n in ['core.js', 'anim.js', 'content1.js', 'content2.js', 'lesson.js', 'lab.js', 'sno_kin.js', 'sno_phys.js', 'sno_walk.js', 'sno_core.js', 'sno_humanoid.js', 'sno_robots.js', 'sno_car.js', 'sno_defs.js', 'app.js']) + '\nApp.init();\n'
css = read('styles.css') + '\n' + read('sno.css')
for part, name in [(js, 'js'), (read('pyworker.js'), 'worker'), (read('pyshim.py'), 'shim'), (css, 'css')]:
    if '</script' in part.lower() or '</style' in part.lower():
        sys.exit(f'closing tag found inside {name}')
html = f'''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Learn ROS 2 with Nodi</title>
<link rel="icon" href="data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 64 64'%3E%3Crect x='6' y='12' width='52' height='40' rx='14' fill='%23F4F8F6' stroke='%231C2B25' stroke-width='4'/%3E%3Crect x='13' y='19' width='38' height='24' rx='8' fill='%23173129'/%3E%3Ccircle cx='25' cy='31' r='4' fill='%235EF2D6'/%3E%3Ccircle cx='39' cy='31' r='4' fill='%235EF2D6'/%3E%3Cpath d='M32 12V5' stroke='%231C2B25' stroke-width='4'/%3E%3Ccircle cx='32' cy='5' r='4' fill='%23F26419'/%3E%3C/svg%3E">
<meta name="description" content="A game-like ROS 2 course with animated lessons, practice, a robot tutor, a virtual robot lab and 3D simulators of the SnoBots robots.">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Fredoka:wght@500;600;700&family=JetBrains+Mono:wght@400;600&family=Lexend:wght@400;500;600&display=swap" rel="stylesheet">
<style>
{css}
</style>
</head>
<body>
<div id="app"><noscript>This learning app needs JavaScript turned on.</noscript></div>
<script type="text/plain" id="py-worker">
{read('pyworker.js')}
</script>
<script type="text/x-python" id="py-shim">
{read('pyshim.py')}
</script>
<script>
{js}
</script>
</body>
</html>
'''
OUT.parent.mkdir(exist_ok=True)
OUT.write_text(html, encoding='utf-8')
print(f'wrote {OUT} ({len(html)/1024:.0f} KB)')
