import matplotlib.pyplot as plt
import csv
import mplleaflet
import argparse


def load_data(path):
    with open(path, newline='') as f:
        reader = csv.reader(f)
        # skip header
        next(reader)
        return [[float(r[0]), float(r[1])] for r in reader if r]


def plt_selection(sources, targets, center, outfile):
    src_lng = [x[0] for x in sources]
    src_lat = [x[1] for x in sources]
    trg_lng = [x[0] for x in targets]
    trg_lat = [x[1] for x in targets]
    center_lng = [x[0] for x in center]
    center_lat = [x[1] for x in center]

    print(src_lng)

    # plt.hold(True)
    plt.plot(src_lng, src_lat, 'b.')
    plt.plot(trg_lng, trg_lat, 'g.')
    plt.plot(center_lng, center_lat, 'r.')
    mplleaflet.show(path=outfile)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='selection plotter')
    parser.add_argument('--sources', '-s', required=True, type=str)
    parser.add_argument('--targets', '-t', required=True, type=str)
    parser.add_argument('--center', '-c', required=True, type=str)
    parser.add_argument('--output', '-o', required=True, type=str)
    args = parser.parse_args()

    src_file = args.sources
    trg_file = args.targets
    center_file = args.center
    output_file = args.output

    sources = load_data(src_file)
    targets = load_data(trg_file)
    center = load_data(center_file)
    plt_selection(sources, targets, center, output_file)
