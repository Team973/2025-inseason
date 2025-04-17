# Download Limelight Config

`.\gradlew downloadLimelightConfig`

Downloads the configuration from a limelight for pipeling 0 as a JSON file.

## Options
- `file-name` The name for the config file. Will be downloaded as `<file-name>.json`
- `limelight` The host name of the limelight you are downloading the config from. The task will search for `http://limelight-<limelight>:5807`. The file will be downloaded at `config/limelight-<limelight>`
