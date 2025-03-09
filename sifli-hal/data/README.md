# Chip Data

The specific data format can be referenced in [build_serde.rs](https://chatgpt.com/build_serde.rs).

## HPSYS_RCC.yaml, HPSYS_CFG.yaml

Generated using the `extract-all` command from [chiptool](https://github.com/embassy-rs/chiptool), with some manual corrections applied. For details, see [sifli-pac/transform/SF32LB52x.yaml](https://github.com/OpenSiFli/sifli-pac/blob/main/transform/SF32LB52x.yaml).

## pinmux.yaml

Generated from [c_array_to_pinmux_yaml.py](../../scripts/c_array_to_pinmux_yaml.py).

Data source: [bf0_pin_const.c](https://github.com/OpenSiFli/SiFli-SDK/blob/main/drivers/cmsis/sf32lb52x/bf0_pin_const.c)

## Others

Derived through LLM processing or manually written.
