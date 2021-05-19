# Communication - rendering bandwidth limit

This test scenario checks that if a team requests too much bandwidth for image
rendering, an error message is sent to the client.

## Manual check

The content of `client_1` should have the following elements:

- It should contain: "Connected to 127.0.0.1"
- It should contain: "ERROR: requested rendering bandwidth is above the limit (371MB/s > 350MB/s)
