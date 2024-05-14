// Function to convert GPS time (in seconds since GPS epoch) to UTC
export default function gpsToUtc({ gpsTimeInSeconds }) {
  const gpsEpoch = new Date(Date.UTC(1980, 0, 6, 0, 0, 0)); // GPS epoch: January 6, 1980
  const gpsUtcOffset = 18; // Current GPS-UTC offset in seconds (as of 2024)

  // Calculate the GPS time in milliseconds
  const gpsTimeInMillis = gpsTimeInSeconds * 1000;

  // Convert GPS time to UTC time in milliseconds
  const utcTimeInMillis =
    gpsEpoch.getTime() + gpsTimeInMillis - gpsUtcOffset * 1000;

  // Create a new Date object for the UTC time
  const utcDate = new Date(utcTimeInMillis);

  return utcDate;
}
