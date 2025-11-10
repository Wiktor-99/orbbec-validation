from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from collections import defaultdict
from pathlib import Path
import cv2
import numpy as np
import struct
import argparse


POINTCLOUD_TOPIC = "/camera/depth/points"
DEPTH_IMAGE_TOPIC = "/camera/depth/image_raw"
COLOR_IMAGE_TOPIC = "/camera/color/image_raw"
TOPICS = [POINTCLOUD_TOPIC, DEPTH_IMAGE_TOPIC, COLOR_IMAGE_TOPIC]


def collect_message_timestamps(mcap_file_path):
    timestamps_by_topic = defaultdict(list)

    with open(mcap_file_path, "rb") as f:
        reader = make_reader(f)
        decoder_factory = DecoderFactory()

        for schema, channel, message in reader.iter_messages():
            if channel.topic not in TOPICS:
                continue

            try:
                decode_fn = decoder_factory.decoder_for(
                    channel.message_encoding, schema
                )
                ros_msg = decode_fn(message.data)
            except Exception as e:
                print(f"Could not decode {channel.topic}: {e}")
                continue

            timestamps_by_topic[channel.topic].append(message.log_time)

    return timestamps_by_topic


def find_synchronized_timestamps(timestamps_by_topic, tolerance_ns):
    pointcloud_times = sorted(timestamps_by_topic[POINTCLOUD_TOPIC])
    depth_times = sorted(timestamps_by_topic[DEPTH_IMAGE_TOPIC])
    color_times = sorted(timestamps_by_topic[COLOR_IMAGE_TOPIC])

    synchronized_triplets = []

    for pointcloud_time in pointcloud_times:
        depth_time = min(depth_times, key=lambda x: abs(x - pointcloud_time))
        color_time = min(color_times, key=lambda x: abs(x - pointcloud_time))

        depth_diff = abs(depth_time - pointcloud_time)
        color_diff = abs(color_time - pointcloud_time)

        if depth_diff <= tolerance_ns and color_diff <= tolerance_ns:
            synchronized_triplets.append((pointcloud_time, depth_time, color_time))

    return synchronized_triplets


def extract_xyz_from_pointcloud(pointcloud_msg):
    xyz_format = "fff"
    point_step = pointcloud_msg.point_step
    data = pointcloud_msg.data

    points = []
    for i in range(0, len(data), point_step):
        point_data = struct.unpack_from(xyz_format, data, i)
        points.append(point_data[:3])

    return np.array(points, dtype=np.float32)


def save_point_cloud_as_pcd(points, file_path):
    with open(file_path, "w") as f:
        f.write(
            f"""# .PCD v0.7
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {len(points)}
HEIGHT 1
POINTS {len(points)}
DATA ascii
"""
        )
        for point in points:
            f.write(f"{point[0]} {point[1]} {point[2]}\n")


def convert_ros_image_to_opencv(ros_image_msg):
    encoding = ros_image_msg.encoding
    height = ros_image_msg.height
    width = ros_image_msg.width
    data = ros_image_msg.data

    if encoding == "16UC1":
        return np.frombuffer(data, dtype=np.uint16).reshape(height, width)
    elif encoding == "32FC1":
        return np.frombuffer(data, dtype=np.float32).reshape(height, width)
    elif encoding == "mono8":
        return np.frombuffer(data, dtype=np.uint8).reshape(height, width)
    elif encoding == "bgr8":
        return np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
    elif encoding == "rgb8":
        img = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    else:
        bytes_per_pixel = len(data) // (height * width)
        if bytes_per_pixel == 1:
            return np.frombuffer(data, dtype=np.uint8).reshape(height, width)
        elif bytes_per_pixel == 2:
            return np.frombuffer(data, dtype=np.uint16).reshape(height, width)
        elif bytes_per_pixel == 3:
            return np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
        else:
            raise ValueError(f"Unsupported encoding: {encoding}")


def create_output_directories(output_directory):
    output_path = Path(output_directory)
    (output_path / "point_clouds").mkdir(parents=True, exist_ok=True)
    (output_path / "depth_images").mkdir(parents=True, exist_ok=True)
    (output_path / "color_images").mkdir(parents=True, exist_ok=True)
    return output_path


def extract_synchronized_data(mcap_file_path, synchronized_timestamps, output_path):
    timestamp_set = {ts for triplet in synchronized_timestamps for ts in triplet}
    saved_count = 0

    with open(mcap_file_path, "rb") as f:
        reader = make_reader(f)
        decoder_factory = DecoderFactory()

        for schema, channel, message in reader.iter_messages():
            if channel.topic not in TOPICS:
                continue

            if message.log_time not in timestamp_set:
                continue

            try:
                decode_fn = decoder_factory.decoder_for(
                    channel.message_encoding, schema
                )
                ros_msg = decode_fn(message.data)
            except Exception as e:
                print(f"Could not decode {channel.topic}: {e}")
                continue

            if channel.topic == POINTCLOUD_TOPIC:
                points = extract_xyz_from_pointcloud(ros_msg)
                file_path = output_path / "point_clouds" / f"{saved_count:06d}.pcd"
                save_point_cloud_as_pcd(points, file_path)

            elif channel.topic == DEPTH_IMAGE_TOPIC:
                image = convert_ros_image_to_opencv(ros_msg)
                file_path = output_path / "depth_images" / f"{saved_count:06d}.png"
                cv2.imwrite(str(file_path), image)

            elif channel.topic == COLOR_IMAGE_TOPIC:
                image = convert_ros_image_to_opencv(ros_msg)
                file_path = output_path / "color_images" / f"{saved_count:06d}.png"
                cv2.imwrite(str(file_path), image)
                saved_count += 1

                if saved_count % 10 == 0:
                    print(f"Processed {saved_count} synchronized sets...")

    return saved_count


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Extract synchronized camera data from MCAP files"
    )
    parser.add_argument("input_mcap", type=str, help="Path to input MCAP file")
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default="sync_mcaps_output",
        help="Output directory for extracted data (default: sync_mcaps_output)",
    )
    parser.add_argument(
        "-t",
        "--tolerance",
        type=float,
        default=15.0,
        help="Synchronization tolerance in milliseconds (default: 15.0)",
    )
    return parser.parse_args()


def main():
    args = parse_arguments()

    mcap_file_path = args.input_mcap
    output_directory = args.output
    sync_tolerance_ns = int(args.tolerance * 1_000_000)

    print(f"Processing MCAP file: {mcap_file_path}")
    print(f"Output directory: {output_directory}")
    print(f"Sync tolerance: {args.tolerance}ms")
    print()

    print("Collecting timestamps from MCAP file...")
    timestamps_by_topic = collect_message_timestamps(mcap_file_path)

    pointcloud_count = len(timestamps_by_topic[POINTCLOUD_TOPIC])
    depth_count = len(timestamps_by_topic[DEPTH_IMAGE_TOPIC])
    color_count = len(timestamps_by_topic[COLOR_IMAGE_TOPIC])

    print(
        f"Found {pointcloud_count} pointcloud, {depth_count} depth, {color_count} color messages"
    )

    print("Finding synchronized message sets...")
    synchronized_timestamps = find_synchronized_timestamps(
        timestamps_by_topic, sync_tolerance_ns
    )
    print(f"Found {len(synchronized_timestamps)} synchronized sets")

    output_path = create_output_directories(output_directory)

    print("Extracting synchronized data...")
    saved_count = extract_synchronized_data(
        mcap_file_path, synchronized_timestamps, output_path
    )

    print(f"Done! Extracted {saved_count} synchronized sets to {output_directory}")


if __name__ == "__main__":
    main()
