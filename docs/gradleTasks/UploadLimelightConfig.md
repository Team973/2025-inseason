# Upload Limelight Config

Uploads the specified JSON configuration file to the on-disk pipeline of a limelight.

## Options
- `file-name` The name of the file that contains the configuration you are uploading. The task will search for `<file-name>.json`
- `limelight` The host name of the limelight you are uploading the config to. The task will search for `http://limelight-<limelight>:5807`. The file will be uploded from `config/limelight-<limelight>`
