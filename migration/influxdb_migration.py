from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from influxdb_client_3 import (
    InfluxDBClient3, write_client_options, WriteOptions, InfluxDBError, Point
)
import datetime
import os
from collections import defaultdict
import pytz
import time
from dotenv import load_dotenv


# # --- OSS config ---
# OSS_URL = "https://xxx.net:8086"
# OSS_TOKEN = os.environ.get("OSS_TOKEN")
# OSS_ORG = "xxx"
# OSS_BUCKET = "xxx"       


# # --- Cloud (destination) config ---
# CLOUD_URL = "https://aws.cloud.com"
# CLOUD_TOKEN = os.environ.get("INFLUXDB_TOKEN")
# CLOUD_ORG = "xxx"
# CLOUD_BUCKET = "xxx"


# Load variables from .env file
load_dotenv()

# --- OSS config ---
OSS_URL = os.getenv("OSS_URL")
OSS_TOKEN = os.getenv("OSS_TOKEN")
OSS_ORG = os.getenv("OSS_ORG")
OSS_BUCKET = os.getenv("OSS_BUCKET")

# --- Cloud (destination) config ---
CLOUD_URL = os.getenv("CLOUD_URL")
CLOUD_TOKEN = os.getenv("INFLUXDB_TOKEN")
CLOUD_ORG = os.getenv("CLOUD_ORG")
CLOUD_BUCKET = os.getenv("CLOUD_BUCKET")



# --- Time range to migrate ---
start_time = datetime.datetime(2026, 7, 1, 0, 0, 0, tzinfo=pytz.UTC)
end_time   = datetime.datetime(2026, 9, 16, 0, 0, 0, tzinfo=pytz.UTC)

# --- Fields you want to migrate ---
ALLOWED_FIELDS = {"Temp",  "U", "I", "P","AmbHmd","AmbTemp","Irr"}

# --- Batch write control ---
BATCH_SIZE = 5000  # number of points per write
SLEEP_BETWEEN_BATCHES = 10  # seconds


# Define callbacks for write responses
def success(self, data: str):
    print(f"Successfully wrote batch: {data}")

def error(self, data: str, exception: InfluxDBError):
    print(f"Failed writing batch: config: {self}, data: {data} due: {exception}")

def retry(self, data: str, exception: InfluxDBError):
    print(f"Failed retry writing batch: config: {self}, data: {data} retry: {exception}")


# --- Connect to OSS ---
client_oss = InfluxDBClient(url=OSS_URL, token=OSS_TOKEN, org=OSS_ORG, verify_ssl=False)
query_api = client_oss.query_api()

print("Querying OSS data...")


# --- Flux query (only the allowed fields) ---
field_filters = " or ".join([f'r["_field"] == "{f}"' for f in ALLOWED_FIELDS])
flux_query = f"""
from(bucket: "{OSS_BUCKET}")
  |> range(start: {start_time.isoformat().replace('+00:00', 'Z')}, stop: {end_time.isoformat().replace('+00:00', 'Z')})
  |> filter(fn: (r) => r["_measurement"] == "ParkData")
  |> filter(fn: (r) => {field_filters})
"""

tables = query_api.query(org=OSS_ORG, query=flux_query)

# --- Parse queried points ---
points = []
count = 0

for table in tables:
    for record in table.records:
        field_name = record.get_field()
        if field_name not in ALLOWED_FIELDS:
            continue

        measurement = "MigratedData_2024-26"


        # Extract tags
        tags = {
            k: v for k, v in record.values.items()
            if k not in ['_time', '_value', '_field', '_measurement', 'result', 'table', '_start', '_stop']
        }

        # Extract fields
        fields = {field_name: record.get_value()}

        # Convert timestamp
        time_val = record.values["_time"]
        if isinstance(time_val, datetime.datetime):
            timestamp = int(time_val.timestamp())
        else:
            timestamp = int(datetime.datetime.fromisoformat(time_val.replace("Z", "+00:00")).timestamp())

        points.append({
            "measurement": measurement,
            "tags": tags,
            "fields": fields,
            "time": timestamp
        })
        count += 1

print(f"Retrieved {count} filtered points from OSS.")



# write_api = cloud_client.write_api(write_options=WriteOptions(batch_size=500, flush_interval=1000))

# write_api.write(bucket=CLOUD_BUCKET, record=points)
# write_api.close()

write_options = WriteOptions(
    batch_size=500,
    flush_interval=10_000,
    jitter_interval=2_000,
    retry_interval=5_000,
    max_retries=5,
    max_retry_delay=30_000,
    exponential_base=2
)

# Create an options dict that sets callbacks and WriteOptions
wco = write_client_options(
    success_callback=success,
    error_callback=error,
    retry_callback=retry,
    write_options=write_options
)

# --- Write to Cloud ---
print("Writing to InfluxDB Cloud Serverless...")

with InfluxDBClient3(
    host=CLOUD_URL,
    token=CLOUD_TOKEN,
    org=CLOUD_ORG,
    database=CLOUD_BUCKET,
    write_client_options=wco
) as cloud_client:
    cloud_client.write(points, write_precision="s")




# # Write in batches to stay within write limits
# for i in range(0, len(points), BATCH_SIZE):
#     batch = points[i:i + BATCH_SIZE]
#     try:
#         cloud_client.write(record=batch, write_precision="s")
#         print(f"Wrote batch {i//BATCH_SIZE + 1} ({len(batch)} points)")
#         time.sleep(SLEEP_BETWEEN_BATCHES)
#     except Exception as e:
#         print(f"Failed to write batch {i//BATCH_SIZE + 1}: {e}")
#         break

cloud_client.close()
client_oss.close()

print("Migration completed successfully!")
