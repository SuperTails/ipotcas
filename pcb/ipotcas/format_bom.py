import csv
from dataclasses import dataclass
from typing import List

# From https://dubiouscreations.com/2019/10/21/using-kicad-with-jlcpcb-assembly-service/


def convert_placement(inpath, outpath, refs):
    with open(inpath, 'r', newline='') as inf:
        csvreader = csv.reader(inf)
        assert(next(csvreader) == ['Ref', 'Val', 'Package', 'PosX', 'PosY', 'Rot', 'Side'])
        with open(outpath, 'w', newline='') as outf:
            csvwriter = csv.writer(outf)
            csvwriter.writerow(['Designator', 'Mid X', 'Mid Y', 'Layer', 'Rotation'])

            for row in csvreader:
                if row[0] in refs:
                    csvwriter.writerow([row[0], row[3], row[4], row[6], row[5]])

def main():
    finalbom = []
    refs = set()

    with open('fab/ipotcas.csv', 'r', newline='') as f:
        c = csv.DictReader(f)

        for row in c:
            if row["JLC P/N"] == "":
                print(f'SKIPPING {row["Reference"]}')
                continue

            for ref in row["Reference"].split(","):
                refs.add(ref)

            finalbom.append([
                f"{row['Value']}: {row['Description']}",
                row["Reference"],
                row["Footprint"],
                row["JLC P/N"]
            ])

    with open('fab/bom-jlc.csv', 'w', newline='') as of:
        csvwriter = csv.writer(of)
        csvwriter.writerow(['Comment', 'Designator', 'Footprint', 'LCSC Part Number'])
        for row in finalbom:
            csvwriter.writerow(row)

    print('Converted bom file to to fab/bom-jlc.csv')

    convert_placement('fab/ipotcas-top-pos.csv', 'fab/placement-jlc.csv', refs)

    print('Converted placement file to fab/placement-jlc.csv')


if __name__ == '__main__':
    main()

