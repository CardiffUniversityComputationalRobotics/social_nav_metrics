import csv
from datetime import datetime

import numpy as np


CSV_FIELDNAMES = [
    "test_number",
    "time",
    "goal_reached",
    "average_sii",
    "average_rmi",
    "total_time",
    "collision_counter",
    "num_nodes",
    "path_irregularity",
    "acc_per_segment",
    "path_length",
]


def import_csv(csvfilename):
    """Open and return all content from a CSV in an array."""
    data = []
    with open(csvfilename, "r", encoding="utf-8", errors="ignore") as scraped:
        reader = csv.reader(scraped, delimiter=",")
        row_index = 0
        for row in reader:
            if row:
                row_index += 1
                columns = [str(row_index), *row]
                data.append(columns)
    return data


def _safe_average(values, default=0.0):
    if hasattr(values, "average"):
        return values.average(default)
    if len(values) == 0:
        return default
    return float(np.average(values))


def save_value_csv(recorder):
    """Save the measured metrics in a new or previously given CSV."""
    recorder.get_logger().info("About to save test measurements.")

    now = datetime.now()
    dt_string = now.strftime("%d/%m/%Y %H:%M:%S")

    last_data = None
    csv_path = f"{recorder.csv_dir_}/{recorder.approach_name_}/{recorder.csv_name_}"

    try:
        csv_read_data = import_csv(csv_path)
        if csv_read_data and csv_read_data[-1][1] != CSV_FIELDNAMES[0]:
            last_data = csv_read_data[-1]
    except OSError:
        recorder.get_logger().warning("Could not open the defined CSV file")

    recorder.get_logger().warning(f"Provided CSV at {csv_path} has been imported.")

    with open(
        csv_path,
        "a",
        newline="",
        encoding="utf-8",
    ) as csvfile_write, open(
        csv_path,
        "r",
        encoding="utf-8",
    ) as csvfile_read:
        reader = csv.reader(csvfile_read)
        writer = csv.DictWriter(csvfile_write, fieldnames=CSV_FIELDNAMES)
        try:
            if next(reader) != CSV_FIELDNAMES:
                writer.writeheader()
        except StopIteration:
            writer.writeheader()

        if last_data is None:
            last_data_index = 1
        else:
            last_data_index = int(last_data[1]) + 1

        if recorder.total_time_ == 0:
            recorder.total_time_ = recorder.current_time_

        writer.writerow(
            {
                "test_number": last_data_index,
                "time": dt_string,
                "goal_reached": recorder.goal_reached_,
                "average_sii": round(_safe_average(recorder.sii_), 4),
                "average_rmi": round(_safe_average(recorder.rmi_), 4),
                "total_time": round(recorder.total_time_, 4),
                "collision_counter": recorder.collision_counter_,
                "num_nodes": int(_safe_average(recorder.num_nodes_)),
                "path_irregularity": round(
                    _safe_average(recorder.path_irregularity_), 4
                ),
                "acc_per_segment": round(
                    _safe_average(recorder.acceleration_per_segment_), 4
                ),
                "path_length": recorder.path_length_,
            }
        )

        recorder.get_logger().warning("Metrics for test saved.")
