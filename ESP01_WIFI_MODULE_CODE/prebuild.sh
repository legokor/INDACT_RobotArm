CONVERTER_SCRIPT="convert_to_cstring.py"
TEXT_OUTPUT_PATH="RobotKar_WIFI/cstring_text"

if [ ! -d "$TEXT_OUTPUT_PATH" ]; then
    mkdir -p "$TEXT_OUTPUT_PATH"
fi

echo This directory contains automatically generated files. Do not modify them. Edit the original text files instead, then run the prebuild script. > $TEXT_OUTPUT_PATH/README.txt

python $CONVERTER_SCRIPT GUI_HTML/robotkar_gui_main.html $TEXT_OUTPUT_PATH/robotkar_gui_main_cstring.h
python $CONVERTER_SCRIPT GUI_HTML/robotkar_back_to_index.html $TEXT_OUTPUT_PATH/robotkar_back_to_index_cstring.h
python $CONVERTER_SCRIPT RobotKar_WIFI/default_json/default_configuration.json $TEXT_OUTPUT_PATH/default_configuration_cstring.h
python $CONVERTER_SCRIPT RobotKar_WIFI/default_json/default_data.json $TEXT_OUTPUT_PATH/default_data_cstring.h
