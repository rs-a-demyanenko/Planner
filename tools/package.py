"""Build an Arduino-installable source ZIP without caches or dependencies."""
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]
version = next(line.split('=', 1)[1] for line in (ROOT / 'library.properties').read_text().splitlines()
               if line.startswith('version='))
output = ROOT / '.build' / ('AxisPlanner-' + version + '.zip')
output.parent.mkdir(exist_ok=True)
with zipfile.ZipFile(output, 'w', zipfile.ZIP_DEFLATED) as archive:
    for name in ('src', 'examples', 'docs', 'README.md', 'LICENSE', 'library.properties', 'library.json', 'keywords.txt'):
        path = ROOT / name
        for file in sorted(path.rglob('*')) if path.is_dir() else [path]:
            if file.is_file():
                archive.write(file, 'AxisPlanner/' + file.relative_to(ROOT).as_posix())
print(output)
