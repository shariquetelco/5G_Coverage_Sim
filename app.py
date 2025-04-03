from flask import Flask, render_template, request
from simulator import calculate_link_budget

app = Flask(__name__)

@app.route("/", methods=["GET", "POST"])
def index():
    if request.method == "POST":
        try:
            # Get form data from user input
            pt = float(request.form["transmitter_power"])
            gt = float(request.form["transmitter_gain"])
            gr = float(request.form["receiver_gain"])
            l = float(request.form["path_loss"])
            pr = float(request.form["receiver_sensitivity"])

            # Calculate link budget
            link_budget = calculate_link_budget(pt, gt, gr, l, pr)
            result = f"Link Budget: {link_budget} dB"
        except ValueError:
            result = "Please enter valid numbers for all fields."

        return render_template("index.html", result=result)

    return render_template("index.html", result="")

if __name__ == "__main__":
    app.run(debug=True)
