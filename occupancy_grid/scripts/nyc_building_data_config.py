"""
@File: nyc_building_data_config.py
@Author: James Swedeen
@Date: January 2023

@brief
Reads in a CSV of building footprints and generates a CSV that the makeOccupancyGridFromBuildingsCsv functions can process.
"""

import argparse
import csv
import pathlib
import os
import shapely
import shapely.wkt
import shapely.ops
import pymap3d.ned

def parseInputArgs():
    des_msg = "Reads in a CSV of building footprints and generates a CSV that the makeOccupancyGridFromBuildingsCsv functions can process."
    parser = argparse.ArgumentParser(description = des_msg)

    parser.add_argument('-i', '--input-file',
                        action = 'store',
                        type=pathlib.Path,
                        required = True,
                        dest = 'input_file',
                        help = 'This is the input file from https://data.cityofnewyork.us/Housing-Development/Building-Footprints/nqwf-w8eh')
    parser.add_argument('-o', '--output-file',
                        action = 'store',
                        type=pathlib.Path,
                        required = False,
                        dest = 'output_file',
                        default = pathlib.Path("output.csv").absolute(),
                        help = 'This is the file that the plotting code will consume')
    parser.add_argument('-mh', '--min-height',
                        action = 'store',
                        type=float,
                        required = False,
                        dest = 'min_height',
                        default = 0.1,
                        help = 'Any buildings that are less then this height in meters will not be included in the occupancy grid')
    parser.add_argument('-rn', '--region-names',
                        action = 'store',
                        type=float,
                        required = False,
                        dest = 'region_names',
                        default = ["Manhattan"],
                        help = 'Names of the regions to include. Possible values are [Manhattan, Bronx, Brooklyn, Queens, Staten Island]')

    args = parser.parse_args()
    return (args.input_file, args.output_file)

def useBuilding(bin_number, region_names):
    assert 1*1e6 < bin_number
    if bin_number < 2*1e6: # Manhattan
        return "Manhattan" in region_names
    if bin_number < 3*1e6: # Bronx
        return "Bronx" in region_names
    if bin_number < 4*1e6: # Brooklyn
        return "Brooklyn" in region_names
    if bin_number < 5*1e6: # Queens
        return "Queens" in region_names
    if bin_number < 6*1e6: # Staten Island
        return "Staten Island" in region_names
    print(bin_number)
    assert False

def main():
    (ifile_name, ofile_name, min_height, region_names) = parseInputArgs()

    with open(ifile_name) as input_file, open(ofile_name, 'w') as output_file:
        input_csv = csv.DictReader(input_file, delimiter=',')
        output_csv = csv.DictWriter(output_file, delimiter=',', lineterminator="\n", fieldnames=['north','east','n_width','e_width','height'])

        output_csv.writeheader()

        # Find average latitude and longitude
        buildings_bounds = [[],[],[],[]]
        buildings_heights = []
        for row in input_csv:
            if useBuilding(int(row['BIN']), region_names) is False:
                continue
            building_height = row['HEIGHTROOF']
            building_elevation = row['GROUNDELEV']
            if ('' == building_height) or ('' == building_elevation):
                continue
            height = (float(building_height) + float(building_elevation)) * 0.3048
            if height < min_height:
                continue
            building_polygon = shapely.wkt.loads(row['the_geom'])
            building_bounds = building_polygon.bounds

            for it in range(4):
                buildings_bounds[it].append(building_bounds[it])
            buildings_heights.append(height)

        avg_lon = (sum(buildings_bounds[0]) + sum(buildings_bounds[2])) / (2*len(buildings_bounds[0]))
        avg_lat = (sum(buildings_bounds[1]) + sum(buildings_bounds[3])) / (2*len(buildings_bounds[0]))

        for (bound_0, bound_1, bound_2, bound_3, height) in zip(buildings_bounds[0], buildings_bounds[1], buildings_bounds[2], buildings_bounds[3], buildings_heights):
            (min_n, min_e, _) = pymap3d.ned.geodetic2ned(lat = bound_1,
                                                         lon = bound_0,
                                                         h = 0,
                                                         lat0 = avg_lat,
                                                         lon0 = avg_lon,
                                                         h0 = 0)
            (max_n, max_e, _) = pymap3d.ned.geodetic2ned(lat = bound_3,
                                                         lon = bound_2,
                                                         h = 0,
                                                         lat0 = avg_lat,
                                                         lon0 = avg_lon,
                                                         h0 = 0)
            center_n = (max_n + min_n) / float(2);
            center_e = (max_e + min_e) / float(2);
            n_width = max_n - min_n
            e_width = max_e - min_e

            output_csv.writerow({'north': center_n,
                                 'east': center_e,
                                 'n_width': n_width,
                                 'e_width': e_width,
                                 'height': height})


if __name__ == "__main__":
   main()

