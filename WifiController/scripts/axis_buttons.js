const buttons = document.querySelectorAll('button');
buttons.forEach((button) => {
    button.addEventListener('click', () => {
        handleButtonClick(button);
    });
});

function handleButtonClick(button) {
    const xhr = new XMLHttpRequest();
    const method = 'GET';
    const url = window.location.origin + '/action/button?btn=' + button.dataset.btn;

    xhr.open(method, url, true);
    xhr.onload = () => {
        const status = xhr.status;
        if (status === 200) {
            console.log(`Button ${button.dataset.btn} was pressed.`);
        } else {
            console.log(`An error occurred while pressing button ${button.dataset.btn}! Received status: ${status}`);
        }
    };
    xhr.onerror = () => {
        console.log(`An error occurred while pressing button ${button.dataset.btn}.`);
    }
    xhr.send();
}