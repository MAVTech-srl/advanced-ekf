import pandas as pd
import plotly.graph_objs as go
from plotly.subplots import make_subplots
from dash import Dash, dcc, html, Input, Output, MATCH, State
import os

"""
This script depends on pandas and dash. Install them in your python venv with pip.
"""

# ---------------- Config ----------------
file_path = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(file_path, "../logs/ekf_log_2026-01-26_12-23-52.csv")
TIME_COLUMN = "timestamp_s"
# ----------------------------------------

CSV_FOLDER = os.path.join(file_path, "../logs")     # folder that contains the *.csv files
DEFAULT_FILE = None                                 # if None the first file alphabetically is used
# ----------------------------------------------------------------------
# 1️⃣ Helper: list available CSV files
# ----------------------------------------------------------------------
def list_csv_files():
    """Return a sorted list of absolute paths for *.csv files in CSV_FOLDER."""
    if not os.path.isdir(CSV_FOLDER):
        return []
    files = [
        f for f in os.listdir(CSV_FOLDER)
        if f.lower().endswith(".csv") and os.path.isfile(os.path.join(CSV_FOLDER, f))
    ]
    files.sort()
    return [os.path.join(CSV_FOLDER, f) for f in files]

# ----------------------------------------------------------------------
# 2️⃣ Load a CSV (called whenever the user picks a new file)
# ----------------------------------------------------------------------
def load_csv(path):
    """Read CSV, coerce timestamp to numeric, drop rows without it."""
    df = pd.read_csv(path, skipinitialspace=True)
    df["timestamp_s"] = pd.to_numeric(df["timestamp_s"], errors="coerce")
    df = df.dropna(subset=["timestamp_s"]).reset_index(drop=True)

    # Get rid of time offset if any
    df["timestamp_s"] = df["timestamp_s"].sub(df["timestamp_s"][0])

    return df


# df = pd.read_csv(CSV_PATH, skipinitialspace=True)

# # Make sure the timestamp column is numeric
# df["timestamp_s"] = pd.to_numeric(df["timestamp_s"], errors="coerce")
# df = df.dropna(subset=["timestamp_s"]).reset_index(drop=True)

# # Get rid of time offset if any
# df["timestamp_s"] = df["timestamp_s"].sub(df["timestamp_s"][0])

# ----------------------------------------------------------------------
# 2️⃣ Define column groups (only keep columns that really exist)
# ----------------------------------------------------------------------
GROUPS = {
    "Position": [
        "pos_x", "pos_y", "pos_z",
    ],
    "Orientation (Quaternion)": [
        "rot_w", "rot_x", "rot_y", "rot_z",
    ],
    "Velocity / Acceleration": [
        "vel_x", "vel_y", "vel_z",
        "acc_x", "acc_y", "acc_z",          
    ],
    "Forces & Torques": [
        "force_x", "force_y", "force_z",
        "torque_x", "torque_y", "torque_z",
    ],
    "Angular speed": [
        "omega_x", "omega_y", "omega_z",
    ],
    "Jitter": [
        "j_x", "j_y", "j_z",
    ],
    "Gravity": [
        "g_x", "g_y", "g_z",
    ],
    "Biases": [
        "bw_x", "bw_y", "bw_z",          
        "ba_x", "ba_y", "ba_z",
    ],
    "Extrinsic positions": [
        "pos_I_L_x", "pos_I_L_y", "pos_I_L_z",
        "pos_O_I_x", "pos_O_I_y", "pos_O_I_z",
    ],
    "Extrinsic rotations": [
        "rot_I_L_w", "rot_I_L_x", "rot_I_L_y", "rot_I_L_z",
        "rot_O_I_w", "rot_O_I_x", "rot_O_I_y", "rot_O_I_z",
    ],
    "Step Timing": [
        "elapsed_one_step_ms",
    ],
}

# ----------------------------------------------------------------------
# 4️⃣ Initialise Dash app
# ----------------------------------------------------------------------
app = Dash(__name__)

# ----------------------------------------------------------------------
# Layout – hidden stores are always present; the Tabs component
# (id="group-tabs") is placed in the layout from the start but starts
# empty until a file is loaded.
# ----------------------------------------------------------------------
app.layout = html.Div([
    html.H2("Grouped CSV Visualizer"),
    # ---------- File selector ----------
    html.Div([
        dcc.Dropdown(
            id="file-selector",
            options=[
                {"label": os.path.basename(p), "value": p}
                for p in list_csv_files()
            ],
            value=DEFAULT_FILE, #or (list_csv_files()[0] if list_csv_files() else None),
            placeholder="Select a CSV file",
            clearable=False,
            style={"flex": "1"},
        ),
        html.Button("Refresh list", id="refresh-button", n_clicks=0,
                    style={"margin-left": "10px"}),
    ], style={"display": "flex", "align-items": "center"}),
    # ---------- Hidden stores ----------
    dcc.Store(id="dataframe-store", data={}),
    dcc.Store(id="groups-store", data={}),
    # ---------- Tabs (empty until a file is chosen) ----------
    dcc.Tabs(id="group-tabs", value=None, children=[]),
    # ---------- Container for dropdown + graph of the active tab ----------
    html.Div(id="tab-content")
])

# ----------------------------------------------------------------------
# 1️⃣ Refresh button – rebuild the file‑selector options
# ----------------------------------------------------------------------
@app.callback(
    Output("file-selector", "options"),
    Output("file-selector", "value"),
    Input("refresh-button", "n_clicks"),
    State("file-selector", "value"),
)
def refresh_file_list(_n_clicks, current_value):
    """Return a fresh list of CSV files. Keep the current selection if it still exists."""
    files = list_csv_files()
    options = [{"label": os.path.basename(p), "value": p} for p in files]

    # If the previously selected file is still present, keep it; otherwise pick the first one.
    new_value = current_value # if current_value in files else (files[0] if files else None)
    return options, new_value

# ----------------------------------------------------------------------
# 1️⃣ When a file is chosen: load data, filter groups, fill the tabs
# ----------------------------------------------------------------------
@app.callback(
    Output("dataframe-store", "data"),
    Output("groups-store", "data"),
    Output("group-tabs", "children"),
    Output("group-tabs", "value"),
    Input("file-selector", "value"),
)
def load_file_and_build_tabs(csv_path):
    if not csv_path:
        return {}, {}, [], None

    df = load_csv(csv_path)

    # Filter groups – keep only columns that exist in this CSV
    filtered_groups = {}
    for name, cols in GROUPS.items():
        present = [c for c in cols if c in df.columns]
        if present:
            filtered_groups[name] = present

    # Build the list of Tab components
    tabs = [
        dcc.Tab(label=name, value=name) for name in filtered_groups.keys()
    ]

    # Choose the first group as the default active tab (if any)
    default_value = tabs[0].value if tabs else None

    return df.to_dict("list"), filtered_groups, tabs, default_value

# ----------------------------------------------------------------------
# 2️⃣ Render dropdown + graph for the selected tab
# ----------------------------------------------------------------------
@app.callback(
    Output("tab-content", "children"),
    Input("group-tabs", "value"),
    State("groups-store", "data"),
)
def render_group_tab(selected_group, groups):
    if not groups or selected_group not in groups:
        return html.P("No columns available for this group.")

    cols = groups[selected_group]

    return html.Div([
        dcc.Dropdown(
            id={"type": "column-selector", "group": selected_group},
            options=[{"label": c, "value": c} for c in cols],
            value=(cols[:4] if selected_group == "Orientation (Quaternion)" else cols[:3]),
            multi=True,
            closeOnSelect=False,      # keep the menu open after each click
            placeholder="Select column(s) to plot",
        ),
        dcc.Graph(
            id={"type": "time-series-graph", "group": selected_group}
        ),
    ])

# ----------------------------------------------------------------------
# 3️⃣ Update the figure when the user changes the column selection
# ----------------------------------------------------------------------
@app.callback(
    Output({"type": "time-series-graph", "group": MATCH}, "figure"),
    Input({"type": "column-selector", "group": MATCH}, "value"),
    State("dataframe-store", "data"),
)
def update_figure(selected_columns, df_dict):
    if not selected_columns:
        return {"data": [], "layout": {"title": "No columns selected"}}

    df = pd.DataFrame(df_dict)

    traces = [
        {
            "x": df["timestamp_s"],
            "y": df[col],
            "mode": "lines",
            "name": col,
        }
        for col in selected_columns
    ]

    return {
        "data": traces,
        "layout": {
            "title": "Selected sensor values over time",
            "xaxis": {"title": "timestamp_s"},
            "yaxis": {"title": "Value", "exponentformat": "none", "tickformat": ".3e"},
            "hovermode": "closest",
        },
    }


# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
# ========================================================================

# # ----------------------------------------------------------------------
# # 3️⃣ Build the Dash app layout
# # ----------------------------------------------------------------------
# app = Dash(__name__)

# app.layout = html.Div([
#     html.H2("CSV Sensor Data – Grouped Viewer"),
#     dcc.Tabs(
#         id="group-tabs",
#         value=list(GROUPS.keys())[0],          # first tab active by default
#         children=[
#             dcc.Tab(label=group_name, value=group_name)
#             for group_name in GROUPS.keys()
#         ],
#     ),
#     # A container that will hold the dropdown + graph for the active tab
#     html.Div(id="tab-content")
# ])

# # ----------------------------------------------------------------------
# # 4️⃣ Render controls for the selected tab
# # ----------------------------------------------------------------------
# @app.callback(
#     Output("tab-content", "children"),
#     Input("group-tabs", "value"),
# )
# def render_group_tab(selected_group):
#     cols = GROUPS[selected_group]

#     if not cols:                               # safety net – should not happen now
#         return html.P(f"No columns available for the “{selected_group}” group.")

#     # NOTE: we give the dropdown and graph a *pattern‑matched* id that
#     # includes the group name. This lets the second callback locate them.
#     return html.Div([
#         dcc.Dropdown(
#             id={"type": "column-selector", "group": selected_group},
#             options=[{"label": c, "value": c} for c in cols],
#             value=(cols[:4] if selected_group == "Orientation (Quaternion)" else cols[:3]),
#             multi=True,
#             placeholder="Select column(s) to plot",
#         ),
#         dcc.Graph(
#             id={"type": "time-series-graph", "group": selected_group}
#         ),
#     ])

# # ----------------------------------------------------------------------
# # 5️⃣ Update the figure when the user changes the selection
# # ----------------------------------------------------------------------
# @app.callback(
#     Output({"type": "time-series-graph", "group": MATCH}, "figure"),
#     Input({"type": "column-selector", "group": MATCH}, "value"),
# )
# def update_figure(selected_columns):
#     # If the user deselects everything, show an empty plot
#     if not selected_columns:
#         return {"data": [], "layout": {"title": "No columns selected"}}

#     traces = [
#         {
#             "x": df["timestamp_s"],
#             "y": df[col],
#             "mode": "lines",
#             "name": col,
#         }
#         for col in selected_columns
#     ]

#     return {
#         "data": traces,
#         "layout": {
#             "title": "Selected sensor values over time",
#             "xaxis": {"title": "timestamp_s"},
#             "yaxis": {"title": "Value"},
#             "hovermode": "closest",
#         },
#     }

if __name__ == "__main__":
    app.run(debug=True)
