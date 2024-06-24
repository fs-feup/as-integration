from google.cloud import storage
import os


def setup_gcs_client():
    """!
    Set up the Google Cloud Storage client.
    """
    os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = (
        "src/cloud_storage/evaluation-data-427214-926daf8b7126.json"
    )
    client = storage.Client()

    return client


def upload_csv_to_bucket(bucket_name, source_file_name, destination_blob_name):
    """!
    Upload a CSV file to a Google Cloud Storage bucket.

    Args:
        bucket_name: Name of the bucket to upload the file to.
        source_file_name: Path to the file to upload.
        destination_blob_name: Name of the file in the bucket.
    """

    client = setup_gcs_client()
    bucket = client.bucket(bucket_name)
    blob = bucket.blob(destination_blob_name)
    blob.upload_from_filename(source_file_name)
    print(f"File {source_file_name} uploaded to {destination_blob_name}.")
