# -*- coding: utf-8 -*-
import dash
from dash import dcc, html, Input, Output
import plotly.graph_objects as go
import numpy as np
import traceback

# ==============================================================================
# 1. KERNEL FISICO (V78 - 10% SNR BOUNDARY THRESHOLD)
# ==============================================================================
class CUI_Physics_Kernel:
    def __init__(self):
        self.FTO = 1.05

    def calculate_spacing(self, N_tot, L_km):
        if N_tot <= 0: return 99999.0
        N_active = N_tot / self.FTO
        if N_active < 1: return 99999.0
        N_side = N_active / 2.0
        return (L_km * 1000.0) / N_side

    def get_R_eff(self, R_teo, k):
        """ Calcola la distanza in cui il segnale scende esattamente al 10% """
        if k <= 0.0: return R_teo
        return R_teo * (1.0 - (0.10 ** (1.0 / k)))

    def calculate_effective_chord(self, y_point, path_y, R_teo, k_factor):
        """
        Calcola la Corda Efficace troncando l'integrazione alla Portata Utile (10%).
        Tutto ciò che sta sotto il 10% di segnale viene considerato rumore bianco (scartato).
        """
        R_eff = self.get_R_eff(R_teo, k_factor)
        dist_perp = abs(y_point - path_y)
        
        # Se siamo fuori dalla Portata Utile del 10%, l'efficacia è zero
        if dist_perp >= R_eff: return 0.0
        
        try:
            # La corda non è più tagliata su R_teo, ma sul nuovo limite R_eff
            x_max = np.sqrt(R_eff**2 - dist_perp**2)
        except:
            return 0.0
            
        # Integrazione numerica solo all'interno della zona utile
        x_vals = np.linspace(0, x_max, 30) 
        r_vals = np.sqrt(x_vals**2 + dist_perp**2)
        
        # L'attenuazione continua a seguire la curva teorica originaria R_teo
        ratio = np.clip(r_vals / R_teo, 0, 1)
        weights = (1.0 - ratio) ** k_factor
        
        efficiency = np.mean(weights)
        return (x_max * 2.0) * efficiency

    def calculate_probability_profile(self, y_range, N, L_km, R_teo, Offset, k_factor):
        S = self.calculate_spacing(N, L_km)
        if S <= 0: S = 99999.0
        
        P_vals = []
        for y in y_range:
            chord_n = self.calculate_effective_chord(y, Offset, R_teo, k_factor)
            chord_s = self.calculate_effective_chord(y, -Offset, R_teo, k_factor)
            
            p_tot = (chord_n + chord_s) / S
            if p_tot > 1.0: p_tot = 1.0
            
            P_vals.append(p_tot * 100.0)
            
        return np.array(P_vals)

    def find_iso_distance(self, N, L_km, R_teo, Offset, Target_Prob, k_factor):
        # Scansiona limitatamente all'Offset + il raggio effettivo
        R_eff = self.get_R_eff(R_teo, k_factor)
        y_scan = np.linspace(0, R_eff + Offset + 20, 100)
        probs = self.calculate_probability_profile(y_scan, N, L_km, R_teo, Offset, k_factor)
        
        if len(probs) > 0 and probs[0] < Target_Prob: 
            return -1.0 
        
        for i in range(len(probs)-1):
            if probs[i] >= Target_Prob and probs[i+1] < Target_Prob:
                return y_scan[i]
        return 0.0

K = CUI_Physics_Kernel()

# ==============================================================================
# 2. INTERFACCIA DASH (V78)
# ==============================================================================
app = dash.Dash(__name__, title="CUI V78 10% THRESHOLD")
server = app.server  # <-- AGGIUNGI QUESTA RIGA

slider_style = {'width': '23%', 'fontSize': '12px', 'padding': '5px'}
row_style = {'display': 'flex', 'justifyContent': 'space-between', 'backgroundColor': 'white', 
             'borderRadius': '8px', 'padding': '10px', 'marginBottom': '10px', 'boxShadow': '0 2px 4px rgba(0,0,0,0.1)'}
CAM_3D = dict(eye=dict(x=-1.5, y=-1.5, z=1.2))

app.layout = html.Div(style={'fontFamily': 'Segoe UI', 'backgroundColor': '#eef2f3', 'height': '100vh', 'padding': '20px'}, children=[
    
    html.H2("CUI V78 - PORTATA UTILE DINAMICA (Soglia Segnale 10%)", style={'textAlign': 'center', 'color': '#2c3e50'}),
    
    # --- RIGA DI CONTROLLO 1: FISICA DEL SENSORE ---
    html.Div(style={**row_style, 'backgroundColor': '#e8f6f3', 'border': '1px solid #1abc9c'}, children=[
        html.Div(style={'width': '100%', 'padding':'5px'}, children=[
            html.Label("📡 FATTORE K (Efficienza Beamforming)", style={'fontWeight':'bold', 'color':'#16a085', 'fontSize': '14px'}),
            dcc.Slider(
                id='k-factor', 
                min=0.0, max=4.0, step=0.1, value=2.0, 
                marks={
                    0.0: {'label': '0.0 (Teorico)', 'style': {'color': '#117a65'}},
                    1.0: {'label': '1.0 (Lineare)', 'style': {'color': '#27ae60'}},
                    2.0: {'label': '2.0 (Passivo)', 'style': {'color': '#2980b9'}},
                    3.0: {'label': '3.0 (Ibrido)', 'style': {'color': '#d35400'}},
                    4.0: {'label': '4.0 (Attivo)', 'style': {'color': '#c0392b', 'fontWeight': 'bold'}}
                },
                tooltip={"placement": "bottom", "always_visible": True}
            )
        ]),
    ]),

    # --- RIGA DI CONTROLLO 2: LOGISTICA FLOTTA ---
    html.Div(style=row_style, children=[
        html.Div(style=slider_style, children=[html.Label("🚁 Flotta Totale (N)"), 
            dcc.Slider(id='n', min=20, max=300, step=10, value=120, marks={20:'20', 150:'150', 300:'300'}, tooltip={"placement": "bottom", "always_visible": True})]),
        
        # Etichetta Dinamica Aggiornata dal Callback
        html.Div(style=slider_style, children=[html.Label(id='label-portata', style={'fontWeight': 'bold', 'color': '#8e44ad'}), 
            dcc.Slider(id='r', min=10, max=100, step=5, value=50, marks={10:'10', 50:'50', 100:'100'}, tooltip={"placement": "bottom", "always_visible": True})]),
        
        html.Div(style=slider_style, children=[html.Label("↔️ Offset Laterale (m)"), 
            dcc.Slider(id='off', min=0, max=100, step=5, value=15, marks={0:'0', 50:'50', 100:'100'}, tooltip={"placement": "bottom", "always_visible": True})]),
        html.Div(style=slider_style, children=[html.Label("🌍 Tratta Cavo (km)"), 
            dcc.Slider(id='len', min=2, max=50, step=2, value=5, marks={2:'2', 25:'25', 50:'50'}, tooltip={"placement": "bottom", "always_visible": True})]),
    ]),

    html.Div(style={'display': 'flex', 'height': '65vh', 'justifyContent': 'space-between'}, children=[
        html.Div(style={'width': '59%', 'height': '100%', 'backgroundColor': 'white', 'borderRadius': '8px', 'padding': '5px'}, children=[
            dcc.Graph(id='g-map', style={'height': '100%'})
        ]),
        html.Div(style={'width': '40%', 'height': '100%', 'display':'flex', 'flexDirection':'column', 'justifyContent':'space-between'}, children=[
            html.Div(style={'height': '49%', 'backgroundColor': 'white', 'borderRadius': '8px'}, children=[
                dcc.Graph(id='g-90', style={'height': '100%'})
            ]),
            html.Div(style={'height': '49%', 'backgroundColor': 'white', 'borderRadius': '8px'}, children=[
                dcc.Graph(id='g-10', style={'height': '100%'})
            ]),
        ]),
    ]),
])

@app.callback(
    [Output('g-map', 'figure'), Output('g-90', 'figure'), Output('g-10', 'figure'), Output('label-portata', 'children')],
    [Input('n', 'value'), Input('r', 'value'), Input('off', 'value'), Input('len', 'value'), Input('k-factor', 'value')]
)
def update(N, R_teo, Offset, L_km, K_val):
    try:
        # Calcolo Portata Utile per la UI
        R_eff = K.get_R_eff(R_teo, float(K_val))
        label_text = f"🎯 Portata Utile >10%: {R_eff:.1f} m (Teorica: {R_teo}m)"

        # 1. MAPPA 2D
        Y_max = R_teo + Offset + 10
        Y = np.linspace(-Y_max, Y_max, 200)
        X = [0, 100]
        
        P = K.calculate_probability_profile(np.abs(Y), N, L_km, R_teo, Offset, float(K_val))
        Z = np.column_stack((P, P))
        
        colors = [
            [0.00, '#000000'], 
            [0.005, '#0000FF'],
            [0.10, '#00FFFF'], 
            [0.40, '#00FF00'], 
            [0.80, '#FFFF00'], 
            [1.00, '#FF0000']
        ]
        
        fig_map = go.Figure(data=go.Heatmap(z=Z, x=X, y=Y, colorscale=colors, zmin=0, zmax=100))
        
        fig_map.add_shape(type="line", x0=0, x1=100, y0=Offset, y1=Offset, line=dict(color="white", width=1, dash="dash"))
        fig_map.add_shape(type="line", x0=0, x1=100, y0=-Offset, y1=-Offset, line=dict(color="white", width=1, dash="dash"))
        fig_map.add_shape(type="line", x0=0, x1=100, y0=0, y1=0, line=dict(color="lime", width=3))
        
        # Disegno dei bordi della Portata Utile
        fig_map.add_shape(type="line", x0=0, x1=100, y0=Offset+R_eff, y1=Offset+R_eff, line=dict(color="gray", width=1, dash="dot"))
        fig_map.add_shape(type="line", x0=0, x1=100, y0=-(Offset+R_eff), y1=-(Offset+R_eff), line=dict(color="gray", width=1, dash="dot"))
        
        fig_map.update_layout(
            title=f"CAMPO PROBABILITÀ (R utile = {R_eff:.1f}m)", 
            margin=dict(l=40, r=20, t=40, b=20), 
            plot_bgcolor='#111', 
            xaxis=dict(showticklabels=False, title="Lunghezza Cavo"), 
            yaxis=dict(title="Distanza Laterale (m)")
        )

        # 2. SUPERFICI 3D
        N_rng = np.linspace(20, 300, 20)
        Off_rng = np.linspace(0, 100, 20)
        X_off, Y_n = np.meshgrid(Off_rng, N_rng)
        Z_90, Z_10 = np.zeros_like(X_off), np.zeros_like(X_off)
        
        for i in range(len(N_rng)):
            for j in range(len(Off_rng)):
                d90 = K.find_iso_distance(N_rng[i], L_km, R_teo, Off_rng[j], 90.0, float(K_val))
                d10 = K.find_iso_distance(N_rng[i], L_km, R_teo, Off_rng[j], 10.0, float(K_val))
                Z_90[i,j] = max(0, d90)
                Z_10[i,j] = d10

        curr90 = K.find_iso_distance(N, L_km, R_teo, Offset, 90.0, float(K_val))
        fig90 = go.Figure(go.Surface(z=Z_90, x=X_off, y=Y_n, colorscale='Reds', showscale=False))
        mk_col = 'yellow' if curr90 >= 0 else 'black'
        fig90.add_trace(go.Scatter3d(x=[Offset], y=[N], z=[max(0, curr90)], mode='markers', marker=dict(size=6, color=mk_col, line=dict(width=2, color='white'))))
        fig90.update_layout(title="ZONA SICURA (>90%)", scene=dict(camera=CAM_3D, xaxis_title="Offset", yaxis_title="N", zaxis_title="m"), margin=dict(l=0, r=0, t=30, b=0))

        curr10 = K.find_iso_distance(N, L_km, R_teo, Offset, 10.0, float(K_val))
        fig10 = go.Figure(go.Surface(z=Z_10, x=X_off, y=Y_n, colorscale='Blues', showscale=False))
        fig10.add_trace(go.Scatter3d(x=[Offset], y=[N], z=[curr10], mode='markers', marker=dict(size=6, color='cyan', line=dict(width=2, color='white'))))
        fig10.update_layout(title="ZONA WARNING (>10%)", scene=dict(camera=CAM_3D, xaxis_title="Offset", yaxis_title="N", zaxis_title="m"), margin=dict(l=0, r=0, t=30, b=0))

        return fig_map, fig90, fig10, label_text

    except Exception as e:
        print("ERRORE:", e)
        traceback.print_exc()
        return go.Figure(), go.Figure(), go.Figure(), "Errore di Calcolo"

if __name__ == '__main__':
    app.run(debug=True, use_reloader=False)