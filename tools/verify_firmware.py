"""Verify manifest SHA256, Intel HEX checksums, addresses, vectors and BIN equality."""
import argparse
import hashlib
import json
from pathlib import Path
import struct


def verify(folder):
    manifest=json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
    for name,digest in manifest['files'].items():
        assert hashlib.sha256((folder/name).read_bytes()).hexdigest()==digest,name
    memory={};upper=0;eof=False;records=0
    for line in (folder/'ak80_9_robotiq.hex').read_text(encoding='ascii').splitlines():
        assert line.startswith(':') and not eof
        raw=bytes.fromhex(line[1:]);count=raw[0];address=int.from_bytes(raw[1:3],'big');kind=raw[3]
        assert len(raw)==count+5 and sum(raw)%256==0
        payload=raw[4:-1];records+=1
        if kind==4:
            assert count==2 and address==0
            upper=int.from_bytes(payload,'big')<<16
        elif kind==0:
            for offset,byte in enumerate(payload):
                absolute=upper+address+offset
                assert 0x08000000<=absolute<0x08100000 and absolute not in memory
                memory[absolute]=byte
        elif kind==1:
            assert count==0 and address==0
            eof=True
        else:
            raise AssertionError(kind)
    binary=(folder/'ak80_9_robotiq.bin').read_bytes()
    assert eof and min(memory)==0x08000000 and len(memory)==len(binary)
    assert bytes(memory[0x08000000+i] for i in range(len(binary)))==binary
    stack,reset=struct.unpack_from('<II',binary)
    assert stack==0x20020000 and reset&1 and 0x08000000<=reset<0x08000000+len(binary)
    return {'passed':True,'hex_checksum_all_records':True,'record_count':records,
        'hex_bin_exact_match':True,'binary_bytes':len(binary),'sha256':manifest['files']}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder',type=Path,default=Path('output/firmware'))
    parser.add_argument('--record',type=Path)
    args=parser.parse_args()
    result=verify(args.folder)
    text=json.dumps(result,indent=2)+'\n'
    if args.record:
        args.record.write_text(text,encoding='utf-8')
    print(text,end='')
