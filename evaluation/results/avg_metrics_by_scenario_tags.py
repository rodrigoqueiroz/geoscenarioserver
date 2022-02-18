#!/usr/bin/env python
import csv

DATASET_LOCATION = 'uni_weber'
SCENARIO_LENGTH = 'full'

with open(f'./{DATASET_LOCATION}/results_{SCENARIO_LENGTH}.csv', mode='r') as csv_file:
	csv_reader = csv.reader(csv_file, delimiter=',')
	next(csv_reader, None)  # skip the headers

	metrics_by_tag = {}
	for row in csv_reader:
		scenario_tags = row[2].split(';')
		for tag in scenario_tags:
			if tag in metrics_by_tag:
				metrics_by_tag[tag]['ade'][0]++
				metrics_by_tag[tag]['frechet'][0]++
				metrics_by_tag[tag]['hausdorff'][0]++
				# change these indices when results.csv is updated
				metrics_by_tag[tag]['ade'][1] += float(row[4])
				metrics_by_tag[tag]['frechet'][1] += float(row[5])
				metrics_by_tag[tag]['hausdorff'][1] += float(row[6])
			else:
				metrics_by_tag[tag] = {'ade': [0, 0.0], 'frechet': [0, 0.0], 'hausdorff': [0, 0.0]}

# calculate averages
with open(f'./{DATASET_LOCATION}/metric_averages_by_tag.csv', 'w') as csv_file:
	csv_writer = csv.writer(csv_file, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
	csv_writer.writerow(['Tag', 'ADE', 'Frechet', 'Hausdorff'])
	for tags in metrics_by_tag.items():
		ade = tags[1]['ade'][1] / tags[1]['ade'][0]
		frechet = tags[1]['frechet'][1] / tags[1]['frechet'][0]
		hausdorff = tags[1]['hausdorff'][1] / tags[1]['hausdorff'][0]
		csv_writer.writerow([tags[0], ade, frechet, hausdorff])
