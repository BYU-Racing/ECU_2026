## Generate hash number to file name mapping
(Note: untested) You should be able to generate a reverse mapping table to
recover file names from the reported hash by running
```bash
$ pio test -e native --filter="test_generate_file_hash_mapping" -v
```