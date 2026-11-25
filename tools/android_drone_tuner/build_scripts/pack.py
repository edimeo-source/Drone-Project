# Unsigned APK = aapt2's base.apk + classes.dex. Stored (uncompressed) entries are padded so
# their data starts on a 4-byte boundary (what zipalign does; Android 11+ requires it for resources.arsc).
import sys, zipfile
base, dex, out = sys.argv[1:4]
src = zipfile.ZipFile(base)
items = [(i, src.read(i.filename)) for i in src.infolist()]
items.append((zipfile.ZipInfo('classes.dex', (1980, 1, 1, 0, 0, 0)), open(dex, 'rb').read()))
items[-1][0].compress_type = zipfile.ZIP_DEFLATED
with open(out, 'wb') as f, zipfile.ZipFile(f, 'w') as z:
    for info, data in items:
        zi = zipfile.ZipInfo(info.filename, (1980, 1, 1, 0, 0, 0))
        zi.compress_type = info.compress_type
        zi.external_attr = 0
        zi.create_system = 0
        if zi.compress_type == zipfile.ZIP_STORED:
            data_start = f.tell() + 30 + len(zi.filename.encode())
            zi.extra = b'\0' * ((4 - data_start % 4) % 4)
        z.writestr(zi, data)
chk = zipfile.ZipFile(out)
for i in chk.infolist():
    off = i.header_offset + 30 + len(i.filename.encode()) + len(i.extra)
    kind = 'stored  ' if i.compress_type == 0 else 'deflated'
    print(f'{kind} {i.filename:42s} data@{off:6d} {"aligned" if i.compress_type or off % 4 == 0 else "NOT ALIGNED"}')
