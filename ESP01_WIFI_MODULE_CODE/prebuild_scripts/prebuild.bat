set CONVERTER_SCRIPT=convert_to_cstring.py
set TEXT_OUTPUT_PATH=RobotArmWifi\cstring_text

if not exist %TEXT_OUTPUT_PATH%\ mkdir %TEXT_OUTPUT_PATH%

echo This directory contains automatically generated files. Do not modify them. Edit the original text files instead, then run the prebuild script.> %TEXT_OUTPUT_PATH%\README.txt

python %CONVERTER_SCRIPT% GUI_HTML\robotarm_gui_main.html %TEXT_OUTPUT_PATH%\robotarm_gui_main_cstring.h
python %CONVERTER_SCRIPT% GUI_HTML\robotarm_back_to_index.html %TEXT_OUTPUT_PATH%\robotarm_back_to_index_cstring.h
python %CONVERTER_SCRIPT% RobotArmWifi\default_json\default_configuration.json %TEXT_OUTPUT_PATH%\default_configuration_cstring.h
python %CONVERTER_SCRIPT% RobotArmWifi\default_json\default_data.json %TEXT_OUTPUT_PATH%\default_data_cstring.h
