import csv

with open('XeY.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    with open('XeY10.csv', mode='w') as employee_file:
    for row in csv_reader:
        if line_count % 10 == 0:
            employee_writer = csv.writer(employee_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            employee_writer.writerow(['John Smith', 'Accounting', 'November'])
            employee_writer.writerow(['Erica Meyers', 'IT', 'March'])
            line_count += 1
    print(f'Processed {line_count} lines.')

    