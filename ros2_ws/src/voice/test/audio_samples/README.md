# Voice Sample Test Data

Place local 16 kHz mono wav files here and add `manifest.tsv`.

`manifest.tsv` format:

```tsv
# wav_path	expected_intent	expected_tool_id
spanner_fetch_01.wav	fetch	spanner_16mm
socket_return_01.wav	return	socket_19mm
```

Audio files are intentionally not committed unless the team decides the dataset
is small enough and cleared for repository use.
