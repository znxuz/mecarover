#!/usr/bin/env python3

import plotly.express as px
import pandas as pd
import sys

filename = sys.argv[1] if len(sys.argv) > 1 else 'timestamps'
data = []
with open(filename, 'r') as file:
    events = {}
    for line in file:
        function, timestamp, event = line.strip().split()
        timestamp = int(timestamp)
        if event == '1':
            if function not in events:
                events[function] = []
            events[function].append(timestamp)
        elif events.get(function):
            start_time = events[function].pop(0)
            data.append({'Function': function, 'Start': start_time, 'End': timestamp})

df = pd.DataFrame(data)
df['Start'] = pd.to_datetime(df['Start'], unit='ms')
df['End'] = pd.to_datetime(df['End'], unit='ms')
df['Duration_Label'] = df['End'] - df['Start']
df['Duration_Label'] = df['Duration_Label'].apply(lambda x: f"{x / pd.Timedelta(milliseconds=1):.0f} µs")

# Sort DataFrame by Function alphabetically before creating the visualization
df = df.sort_values('Function')

fig = px.timeline(df, x_start="Start", x_end="End", y="Function", color="Function", title="Execution Timeline (Microseconds)")

for idx, row in df.iterrows():
    fig.add_annotation(
        x=row['Start'] + (row['End'] - row['Start']) / 2,
        y=row['Function'],
        text=row['Duration_Label'],
        showarrow=False,
        xanchor="center",
        yanchor="middle",
        font=dict(color="black", size=12),
        bgcolor="white",
        opacity=0.7
    )

# Set initial x-axis range to show and allow scrolling
fig.update_xaxes(
    rangeslider_visible=True,
    range=[df['Start'].min(), df['Start'].min() + pd.Timedelta(seconds=5)]
)

fig.update_layout(
    xaxis=dict(type="date", tickformat='%s%L'),
    template="ggplot2",
    xaxis_title="Time (Microseconds)",
    yaxis_title="Thread/Function",
    title_x=0.5,
    margin=dict(l=20, r=30, t=50, b=20),
    hovermode="x unified",
    font=dict(size=10)
)

# fig.show()
# use firefox, because brave somehow tries to launch kwallet
fig.show(renderer="browser")  # Forces system default browser
