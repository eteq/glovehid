#!/usr/bin/env python3

import json
from pathlib import Path
from subprocess import check_call, check_output

import uf2conv

def main():
    working_dir = Path('.')
    project_name = 'unknown'

    fpath = Path(__file__)
    if fpath.parent.name == 'deploy_scripts':
        # assume this is invoked from inside the project
        working_dir = fpath.parents[1]
        project_name = working_dir.stem

    metadata = json.loads(check_output('cargo metadata --format-version=1'.split(), cwd=working_dir))
    target_dir = Path(metadata['target_directory'])

    binpath = target_dir / (project_name + '.bin')
    uf2path = binpath.with_suffix('.uf2')

    print(f'Doing objcopy to create bin file {binpath}')

    check_call(f'cargo objcopy  -- -O binary {binpath}'.split(), cwd=working_dir)

    # use memory.x file to try to determine where to start flashing
    flash_origin = None
    memoryx = working_dir / 'memory.x'
    if memoryx.is_file():
        with memoryx.open('r') as f:
            for l in f:
                if l.strip().startswith('FLASH :'):
                    for item in l.split('FLASH :')[1].split(','):
                        if ' = ' in item:
                            keyval = item.strip().split(' = ')
                            if len(keyval) == 2:
                                key, val = keyval
                                if key == 'ORIGIN':
                                    flash_origin = int(val, 0)
    if flash_origin is None:
        print('Could not use memory.x to determine the offset. Using default of 0x0')
        uf2conv.appstartaddr = 0x0
    else:
        uf2conv.appstartaddr = flash_origin
    uf2conv.familyid = 0xADA52840 #NRF52840

    with binpath.open('rb') as fr:
        with uf2path.open('wb') as fw:
            fw.write(uf2conv.convert_to_uf2(fr.read()))

    print(f'Generated {uf2path}')

    return uf2path

if __name__ == '__main__':
    main()