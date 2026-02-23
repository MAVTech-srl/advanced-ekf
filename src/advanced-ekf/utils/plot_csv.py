import os
import pandas as pd
from dash import Dash, dcc, html, Input, Output, State, MATCH, ALL, callback_context, exceptions
import plotly.graph_objs as go
from plotly.subplots import make_subplots

# ----------------------------------------------------------------------
# CONFIG
# ----------------------------------------------------------------------
file_path = os.path.dirname(os.path.abspath(__file__))
TIME_COLUMN = "timestamp_s"

CSV_FOLDER = os.path.join(file_path, "../logs")   # folder that contains the *.csv files
DEFAULT_FILE = None                               # if None the first file alphabetically is used

# ----------------------------------------------------------------------
# Helper: list CSV files
# ----------------------------------------------------------------------
def list_csv_files():
    if not os.path.isdir(CSV_FOLDER):
        return []
    files = [
        f for f in os.listdir(CSV_FOLDER)
        if f.lower().endswith(".csv") and os.path.isfile(os.path.join(CSV_FOLDER, f))
    ]
    files.sort()
    return [os.path.join(CSV_FOLDER, f) for f in files]

# ----------------------------------------------------------------------
# Load CSV
# ----------------------------------------------------------------------
def load_csv(path):
    df = pd.read_csv(path, skipinitialspace=True)
    df["timestamp_s"] = pd.to_numeric(df["timestamp_s"], errors="coerce")
    df = df.dropna(subset=["timestamp_s"]).reset_index(drop=True)
    df["timestamp_s"] = df["timestamp_s"].sub(df["timestamp_s"][0])   # zero‑offset
    return df

# ----------------------------------------------------------------------
# Column groups (static – will be filtered after loading)
# ----------------------------------------------------------------------
GROUPS = {
    "Position": ["pos_x", "pos_y", "pos_z"],
    "Orientation (Quaternion)": ["rot_x", "rot_y", "rot_z", "rot_w"],
    "Velocity": ["vel_x", "vel_y", "vel_z"],
    "Forces & Torques": ["force_x", "force_y", "force_z", "torque_x", "torque_y", "torque_z"],
    "Angular speed": ["omega_x", "omega_y", "omega_z"],
    "Jitter": ["j_x", "j_y", "j_z"],
    "Gravity": ["g_x", "g_y", "g_z"],
    "Biases": ["bw_x", "bw_y", "bw_z", "ba_x", "ba_y", "ba_z", "btau_x", "btau_y", "btau_z"],
    "Extrinsic positions": ["pos_I_L_x", "pos_I_L_y", "pos_I_L_z", "pos_O_I_x", "pos_O_I_y", "pos_O_I_z"],
    "Extrinsic rotations": ["rot_I_L_x", "rot_I_L_y", "rot_I_L_z", "rot_I_L_w", "rot_O_I_x", "rot_O_I_y", "rot_O_I_z", "rot_O_I_w"],
    "IMU Acceleration": ["imu_acc_x", "imu_acc_y", "imu_acc_z"],
    "IMU Angular speed": ["imu_omega_x", "imu_omega_y", "imu_omega_z"],
    "PX4 Position": ["px4_position_x", "px4_position_y", "px4_position_z"],
    "PX4 Velocity": ["px4_velocity_x", "px4_velocity_y", "px4_velocity_z"],
    "PX4 Orientation": ["px4_orientation_x", "px4_orientation_y", "px4_orientation_z", "px4_orientation_w"],
    "Step Timing": ["elapsed_one_step_ms"],
}

# ----------------------------------------------------------------------
# Initialise Dash
# ----------------------------------------------------------------------
app = Dash(__name__, suppress_callback_exceptions=True)

# ----------------------------------------------------------------------
# Layout – includes a hidden “compare‑mode” dropdown that only appears
# in the special tab
# ----------------------------------------------------------------------
app.layout = html.Div([
    html.H2("Grouped CSV Visualizer"),
    # ---- File selector row ----
    html.Div([
        dcc.Dropdown(
            id="file-selector",
            options=[{"label": os.path.basename(p), "value": p} for p in list_csv_files()],
            value=DEFAULT_FILE,
            placeholder="Select a CSV file",
            clearable=False,
            style={"flex": "1"},
        ),
        html.Button("Refresh list", id="refresh-button", n_clicks=0,
                    style={"margin-left": "10px"}),
    ], style={"display": "flex", "align-items": "center"}),

    # ---- Hidden stores ----
    dcc.Store(id="dataframe-store", data={}),
    dcc.Store(id="groups-store", data={}),

    # ---- Tabs (normal groups + compare tab) ----
    dcc.Tabs(id="group-tabs", value=None, children=[]),

    # ---- Content area (normal view or compare view) ----
    html.Div(id="tab-content")
])

# ----------------------------------------------------------------------
# Refresh button – rebuild file‑selector options
# ----------------------------------------------------------------------
@app.callback(
    Output("file-selector", "options"),
    Output("file-selector", "value"),
    Input("refresh-button", "n_clicks"),
    State("file-selector", "value"),
)
def refresh_file_list(_n_clicks, current_value):
    files = list_csv_files()
    options = [{"label": os.path.basename(p), "value": p} for p in files]
    new_value = current_value if current_value in files else (files[0] if files else None)
    return options, new_value

# ----------------------------------------------------------------------
# Load CSV → store dataframe & filtered groups → build tabs
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

    # Keep only groups that have at least one column present in the file
    filtered_groups = {}
    for name, cols in GROUPS.items():
        present = [c for c in cols if c in df.columns]
        if present:
            filtered_groups[name] = present

    # Normal per‑group tabs
    normal_tabs = [
        dcc.Tab(label=name, value=name) for name in filtered_groups.keys()
    ]

    # Extra tab for comparison
    compare_tab = dcc.Tab(label="Compare groups", value="compare")

    all_tabs = normal_tabs + [compare_tab]

    # Default active tab = first normal group (or compare if none)
    default_value = all_tabs[0].value if all_tabs else None

    return df.to_dict("list"), filtered_groups, all_tabs, default_value

# ----------------------------------------------------------------------
# Render content for the selected tab
# ----------------------------------------------------------------------
@app.callback(
    Output("tab-content", "children"),
    Input("group-tabs", "value"),
    State("groups-store", "data"),
)
def render_tab(selected_tab, groups):
    if selected_tab != "compare":
        if not groups or selected_tab not in groups:
            return html.P("No columns available for this group.")

        cols = groups[selected_tab]

        return html.Div([
            dcc.Dropdown(
                id={"type": "column-selector", "group": selected_tab},
                options=[{"label": c, "value": c} for c in cols],
                value=(cols[:4] if selected_tab == "Orientation (Quaternion)" or selected_tab == "PX4 Orientation" or selected_tab == "Extrinsic rotations" else cols[:3]),
                multi=True,
                closeOnSelect=False,
                placeholder="Select column(s) to plot",
            ),
            dcc.Graph(
                id={"type": "time-series-graph", "group": selected_tab}
            ),
        ])

    # ---- compare‑tab layout -------------------------------------------------
    compare_groups_dropdown = dcc.Dropdown(
        id="compare-groups-selector",
        options=[{"label": g, "value": g} for g in groups.keys()],
        value=[],
        multi=True,
        placeholder="Select groups to compare",
        style={"margin-bottom": "10px"},
    )

    per_group_selectors = html.Div(id="per-group-selectors",
                                   style={"margin-bottom": "10px"})
    column_store = dcc.Store(id="compare-columns-store", data={})
    compare_graph = dcc.Graph(id="compare-graph")

    return html.Div([
        compare_groups_dropdown,
        per_group_selectors,
        column_store,
        compare_graph,
    ])


# ----------------------------------------------------------------------
# Build a column‑selector for each group that has been chosen for comparison
# ----------------------------------------------------------------------
@app.callback(
    Output("per-group-selectors", "children"),
    Output("compare-columns-store", "data", allow_duplicate=True),
    Input("compare-groups-selector", "value"),
    State("groups-store", "data"),
    State("compare-columns-store", "data"),
    prevent_initial_call=True,
)
def build_group_column_selectors(selected_groups, groups, stored_columns):
    """
    - Generates a dropdown for each selected group.
    - Returns a dict {group: [selected columns]} that is stored in
      `compare-columns-store`.  If a group is newly added we default to
      *all* its columns; if it already existed we keep the previous
      selection (as long as the columns still exist).
    """
    if not selected_groups:
        return [], {}

    selectors = []
    new_store = {}

    for grp in selected_groups:
        cols = groups.get(grp, [])
        # Preserve previous selection if possible
        prev = stored_columns.get(grp, cols) if stored_columns else cols
        valid = [c for c in prev if c in cols] or cols

        new_store[grp] = valid

        selectors.append(
            html.Div([
                html.Label(f"{grp} columns:", style={"margin-right": "8px"}),
                dcc.Dropdown(
                    id={"type": "compare-column-selector", "group": grp},
                    options=[{"label": c, "value": c} for c in cols],
                    value=valid,
                    multi=True,
                    placeholder="Select columns",
                    style={"min-width": "200px"},
                )
            ], style={"display": "inline-block",
                      "margin-right": "20px",
                      "margin-bottom": "8px"})
        )

    return selectors, new_store

# ----------------------------------------------------------------------
# 3️⃣ Callback that updates the hidden store when any per‑group
#     column‑selector changes.
# ----------------------------------------------------------------------
@app.callback(
    Output("compare-columns-store", "data", allow_duplicate=True),
    Input({"type": "compare-column-selector", "group": ALL}, "value"),
    State({"type": "compare-column-selector", "group": ALL}, "id"),
    State("compare-columns-store", "data"),
    prevent_initial_call=True,
)
def update_compare_store(all_selected_vals, all_ids, store):
    """
    *all_selected_vals* – list of the selected column lists, one entry per
    dropdown.
    *all_ids* – list of the component ids that produced those values.
    The callback rebuilds the dictionary {group: [columns]} and writes it
    back to the hidden store.
    """
    # Initialise the store if it does not exist yet
    store = store or {}

    # Walk through the parallel lists and update the entry for each group
    for cols, comp_id in zip(all_selected_vals, all_ids):
        group = comp_id["group"]
        store[group] = cols or []          # empty list = “nothing selected”

    return store


# ----------------------------------------------------------------------
# Update normal per‑group figure (unchanged)
# ----------------------------------------------------------------------
@app.callback(
    Output({"type": "time-series-graph", "group": MATCH}, "figure"),
    Input({"type": "column-selector", "group": MATCH}, "value"),
    State("dataframe-store", "data"),
)
def update_normal_figure(selected_columns, df_dict):
    if not selected_columns:
        return {"data": [], "layout": {"title": "No columns selected"}}
    df = pd.DataFrame(df_dict)
    traces = [
        {"x": df["timestamp_s"], "y": df[col], "mode": "lines", "name": col}
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

# ----------------------------------------------------------------------
# 3️⃣ Build stacked figure for the selected comparison groups
# ----------------------------------------------------------------------
DEFAULT_PALETTE = [
    "#636EFA", "#EF553B", "#00CC96", "#AB63FA"
]

@app.callback(
    Output("compare-graph", "figure"),
    Input("compare-groups-selector", "value"),
    Input("compare-columns-store", "data"),
    State("dataframe-store", "data"),
    prevent_initial_call=True,
)
def build_compare_figure(selected_groups, columns_store, df_dict):
    """Create a vertical stack of sub‑plots – one per selected group,
    using a deterministic colour order for all columns."""
    if not selected_groups:
        return {"data": [], "layout": {"title": "Select groups to compare"}}

    df = pd.DataFrame(df_dict)

    # --------------------------------------------------------------
    # 1️⃣ Build a **global** colour map
    # --------------------------------------------------------------
    # Gather every column that will be plotted (across all groups)
    all_selected_cols = []
    for grp in selected_groups:
        all_selected_cols.extend(columns_store.get(grp, []))

    # Preserve the order in which columns first appear, then assign colours
    seen = set()
    ordered_cols = []
    for col in all_selected_cols:
        if col not in seen:
            seen.add(col)
            ordered_cols.append(col)

    # colour_index = {
    #     col: DEFAULT_PALETTE[i % len(DEFAULT_PALETTE)]
    #     for i, col in enumerate(ordered_cols)
    # }

    # --------------------------------------------------------------
    # 2️⃣ Build the figure – one subplot per group
    # --------------------------------------------------------------
    rows = len(selected_groups)
    fig = make_subplots(
        rows=rows,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.04,
        subplot_titles=selected_groups,
    )

    for i, grp in enumerate(selected_groups, start=1):
        if grp == "Orientation (Quaternion)" or grp == "PX4 Orientation" or grp == "Extrinsic rotations":
            DEFAULT_PALETTE = [
                "#636EFA", "#EF553B", "#00CC96", "#AB63FA"
            ]
        else:
            DEFAULT_PALETTE = [
                "#636EFA", "#EF553B", "#00CC96"
            ]
        colour_index = {
            col: DEFAULT_PALETTE[i % len(DEFAULT_PALETTE)]
            for i, col in enumerate(ordered_cols)
        }
        cols = columns_store.get(grp, [])
        for col in cols:
            fig.add_trace(
                go.Scatter(
                    x=df["timestamp_s"],
                    y=df[col],
                    mode="lines",
                    name=col,
                    line=dict(color=colour_index[col]),
                    showlegend=False,
                ),
                row=i,
                col=1,
            )

        fig.update_yaxes(
            title_text=grp,
            row=i,
            col=1,
            automargin=True,
        )

    # --------------------------------------------------------------
    # 3️⃣ Layout (height per subplot)
    # --------------------------------------------------------------
    fig.update_layout(
        height=340 * rows,
        title_text="Comparison of selected groups",
        hovermode="x unified",
        showlegend=False,
    )
    return fig


# ----------------------------------------------------------------------
# Run the server
# ----------------------------------------------------------------------
if __name__ == "__main__":
    app.run(debug=True)
