# ECU_2026
ECU for 2025-2026 FSAE EV Car


### ECU V1
For testing the motor on flat car only


### ECU V2
Saftey standard compliant


### ECU V3
Rules compliant

## Running tests
Go to the platformIO sidebar, go to `native_tests`, open the `Advanced` folder, and click `Test`. Or if you love terminals, run
```bash
$ pio test -e native_tests
```

### Generate hash number to file name mapping
(Note: untested) You should be able to generate a reverse mapping table to
recover file names from the reported hash by running
```bash
$ pio test -e native_tests --filter="test_generate_file_hash_mapping" -v
```
