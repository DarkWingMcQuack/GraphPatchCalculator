import matplotlib.pyplot as plt
import json
import mplleaflet
import argparse


def load_sources_data(path):
    with open(path, newline='') as f:
        data = json.load(f)
        print(data)
        sources = data["source_coords"]
        source_lngs = [x[1] for x in sources]
        source_lats = [x[0] for x in sources]
        return source_lngs, source_lats


def load_targets_data(path):
    with open(path, newline='') as f:
        data = json.load(f)
        targets = data["target_coords"]
        target_lngs = [x[1] for x in targets]
        target_lats = [x[0] for x in targets]
        return target_lngs, target_lats


def load_center_data(path):
    with open(path, newline='') as f:
        data = json.load(f)
        center = data["center_coords"]
        return [center[1]], [center[0]]


def plt_selection(infile, outfile):
    src_lng, src_lat = load_sources_data(infile)
    trg_lng, trg_lat = load_targets_data(infile)
    center_lng, center_lat = load_center_data(infile)

    # plt.hold(True)
    plt.plot(src_lng, src_lat, 'b.')
    plt.plot(trg_lng, trg_lat, 'g.')
    plt.plot(center_lng, center_lat, 'r.')
    mplleaflet.show(path=outfile)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='selection plotter')
    parser.add_argument('--file', '-f', required=True, type=str)
    parser.add_argument('--output', '-o', required=True, type=str)
    args = parser.parse_args()

    infile = args.file
    outfile = args.output

    plt_selection(infile, outfile)
