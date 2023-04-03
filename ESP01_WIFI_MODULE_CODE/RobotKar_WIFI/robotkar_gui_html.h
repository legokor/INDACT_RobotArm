/**
 *******************************************************************************
 * @file robotkar_gui_html.h
 *
 * @date Jan 20, 2023
 * @author Varga Peter
 *******************************************************************************
 * @brief This file contains the HTML pages used by the webserver.
 *******************************************************************************
 */

#ifndef ROBOTKAR_GUI_HTML_H_
#define ROBOTKAR_GUI_HTML_H_

static const char gui_main_page[] PROGMEM = "\
<!DOCTYPE html>\
<html>\
\
<head>\
    <title>INDACT Robotkar</title>\
    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\
    <style>\
        h1 {\
            padding: 10px;\
            margin: 10px;\
            text-align: center;\
        }\
\
        #status,\
        #coordinates {\
            font-size: 24px;\
            font-family: 'Courier New', Courier, monospace;\
            background-color: lightgray;\
            padding: 10px;\
            margin: 10px;\
        }\
\
        .controls {\
            border: 2px solid black;\
            border-collapse: collapse;\
            width: 100%;\
            margin: 10px;\
        }\
\
        .controls th,\
        .controls td {\
            border-bottom: 1px solid #ddd;\
            width: 50%;\
        }\
\
        .controls th {\
            font-size: 24px;\
        }\
\
        #position {\
            padding: 10px;\
            font-size: 24px;\
            text-align: center;\
        }\
\
        button {\
            border: 1px solid black;\
            border-radius: 8px;\
            color: black;\
            width: 80px;\
            margin: 4px;\
            padding: 10px;\
            text-align: center;\
            font-family: 'Courier New', Courier, monospace;\
            font-size: 28px;\
            cursor: pointer;\
        }\
\
        button:hover {\
            background-color: darkgray;\
            color: white;\
        }\
    </style>\
    <script>\
        function requestData() {\r\n\
            // Send a GET request to the URL of the server\r\n\
            const xhr = new XMLHttpRequest();\r\n\
            const method = 'GET';\r\n\
            const url = window.location.origin + '/data';\r\n\
\r\n\
            xhr.open(method, url, true);\r\n\
            xhr.responseType = 'json';\r\n\
            xhr.onload = () => {\r\n\
                const status = xhr.status;\r\n\
                if (status === 200) {\r\n\
                    // The request has been completed successfully\r\n\
                    const data = xhr.response;\r\n\
                    console.log(data);\r\n\
\
                    // JSON\r\n\
                    // {\r\n\
                    //     status: ...,\r\n\
                    //     csys: {\r\n\
                    //         A: ...,\r\n\
                    //         B: ...,\r\n\
                    //         C: ... \r\n\
                    //     },\r\n\
                    //     position: ...\r\n\
                    // }\r\n\
\r\n\
                    // Use data, change the DOM\r\n\
                    if (data.status === '') {\r\n\
                        document.getElementById('status_msg').innerHTML = '( ... )';\r\n\
                    } else {\r\n\
                        document.getElementById('status_msg').innerHTML = data.status;\r\n\
                    }\r\n\
\r\n\
                    document.getElementById('axis_A').innerHTML = data.csys.A + ' tengely';\r\n\
                    document.getElementById('axis_B').innerHTML = data.csys.B + ' tengely';\r\n\
                    document.getElementById('axis_C').innerHTML = data.csys.C + ' tengely';\r\n\
\r\n\
                    if (data.position === '') {\r\n\
                        document.getElementById('status_msg').innerHTML = 'Poz&iacute;ci&oacute;: ( ... )';\r\n\
                    } else {\r\n\
                        document.getElementById('position').innerHTML = 'Poz&iacute;ci&oacute;: ' + data.position;\r\n\
                    }\r\n\
                } else {\r\n\
                    // There has been an error with the request!\r\n\
                    console.log('Something went wrong! Received status: ' + status);\r\n\
                }\r\n\
            };\r\n\
            xhr.onerror = () => {\r\n\
                console.log('An error occured.');\r\n\
            }\r\n\
            xhr.send();\r\n\
        }\r\n\
\r\n\
        document.addEventListener('DOMContentLoaded', () => {\r\n\
            requestData();\r\n\
\r\n\
            // Send an action request to the server if a button is pressed.\r\n\
            document.querySelectorAll('button').forEach((button) => {\r\n\
                button.onclick = () => {\r\n\
                    // Send a GET request to the URL of the server\r\n\
                    const xhr = new XMLHttpRequest();\r\n\
                    const method = 'GET';\r\n\
                    const url = window.location.origin + '/action/' + button.dataset.action;\r\n\
\r\n\
                    xhr.open(method, url, true);\r\n\
                    xhr.onload = () => {\r\n\
                        const status = xhr.status;\r\n\
                        if (status === 200) {\r\n\
                            // The request has been completed successfully\r\n\
                            console.log('Data sent and response loaded.');\r\n\
                        } else {\r\n\
                            // There has been an error with the request!\r\n\
                            console.log('Something went wrong! Received status: ' + status);\r\n\
                        }\r\n\
                    };\r\n\
                    xhr.onerror = () => {\r\n\
                        console.log('An error occured.');\r\n\
                    }\r\n\
                    xhr.send();\r\n\
                }\r\n\
            });\r\n\
\r\n\
            // Get data from the webserver (WiFi module)\r\n\
            const intervalId = setInterval(requestData, 4000);\r\n\
        });\r\n\
    </script>\
</head>\
\
<body>\
    <h1>INDACT Robotkar</h1>\
    <div id=\"status\">\
        <b>&Aacute;llapot: </b><br>\
        <div id=\"status_msg\">...</div>\
    </div>\
    <table class=\"controls\">\
        <tr>\
            <td id=\"position\" colspan=\"2\">Poz&iacute;ci&oacute;: ...</td>\
        </tr>\
        <tr>\
            <th>\
                <div id=\"axis_A\">A tengely</div>\
            </th>\
            <td>\
                <button data-action=\"ap\">+</button>\
                <button data-action=\"am\">-</button>\
            </td>\
        </tr>\
        <tr>\
            <th>\
                <div id=\"axis_B\">B tengely</div>\
            </th>\
            <td>\
                <button data-action=\"bp\">+</button>\
                <button data-action=\"bm\">-</button>\
            </td>\
        </tr>\
        <tr>\
            <th>\
                <div id=\"axis_C\">C tengely</div>\
            </th>\
            <td>\
                <button data-action=\"cp\">+</button>\
                <button data-action=\"cm\">-</button>\
            </td>\
        </tr>\
        <tr>\
            <th>Alaphelyzet</th>\
            <td>\
                <button data-action=\"hx\">H</button>\
            </td>\
        </tr>\
    </table>\
    <div id=\"coordinates\">\
        <b>Koordin&aacute;ta rendszer v&aacute;lt&aacute;s:</b>\
        (r, &phi;, z) vagy (x, y, z)\
        <button data-action=\"kx\">C</button>\
    </div>\
</body>\
\
</html>\
";

static const char back_to_index_page[] PROGMEM = "\
<!DOCTYPE html>\
<html>\
<head>\
    <title>INDACT Robotkar</title>\
    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\
    <style>\
        div {\
            margin: 10px;\
            padding: 10px;\
            background-color: lightgray;\
            text-align: center;\
            font-size: 20px;\
        }\
        a {\
            color: blueviolet;\
            font-size: 24px;\
        }\
    </style>\
</head>\
<body>\
    <div>\
        <h1>Hiba!</h1>\
        <p>Az elk&uuml;ld&ouml;tt k&eacute;r&eacute;s hib&aacute;s vagy nem siker&uuml;lt felismerni.</p>\
        <p><a href=\"/\">Vissza a vez&eacute;rl&odblac; oldalra</a></p>\
    </div>\
</body>\
</html>\
";

#endif /* ROBOTKAR_GUI_HTML_H_ */
