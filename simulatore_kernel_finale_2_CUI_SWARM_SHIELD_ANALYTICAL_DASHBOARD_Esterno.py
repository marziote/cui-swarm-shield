# -*- coding: utf-8 -*-
import dash
from dash import dcc, html, Input, Output, ctx, no_update
import plotly.graph_objects as go
import numpy as np

# ==============================================================================
# 1. KERNEL MATEMATICO (V38 - FINAL LEGEND FIX)
# ==============================================================================
class CUI_Kernel:
    def __init__(self):
        # Parametri Operativi
        self.D_std = 8900.0      
        self.T_miss = 3.0        
        self.T_swap = 0.1        
        self.P_drone_avg = 0.5   
        self.P_staz_hotel = 3.0  
        self.BW_stream = 20.0    
        self.N_int_safety = 2    
        
        # Economici Base
        self.C_chg = 5000        
        self.C_kwh = 400         
        self.C_srv = 8000        
        self.C_cable_base = 150  
        self.C_cable_mw = 100    
        self.C_mob_vessel = 500  
        self.C_BUOY_POPUP = 25000 
        
        self.PHI_OVERLAP = 2.0 

        # Init Limiti per la scala colori fissa (CAPEX)
        c_min, _, _, _, _, _, _, _, _, _ = self.calculate_system_complete(10, 5, 1, 5, 500, 0.5)
        c_max, _, _, _, _, _, _, _, _, _ = self.calculate_system_complete(10, 100, 14, 40, 3000, 4.0)
        
        self.GLOBAL_MIN_CAPEX = c_min
        self.GLOBAL_MAX_CAPEX = c_max

    def get_fto_vehicle(self): return 1 + (self.T_swap / self.T_miss)
    def get_fto_battery(self, t_charge): return 1 + (t_charge / self.T_miss)
    def knots_to_ms(self, knots): return knots * 0.514444
    def knots_to_m_min(self, knots): return knots * 30.8667

    def solve_blind_time(self, N_tot_uuv, V_kts, R_sens):
        V_ms = self.knots_to_ms(V_kts)
        FTO_v = self.get_fto_vehicle()
        N_att_phys = N_tot_uuv / FTO_v 
        if N_att_phys <= 0: return 0
        N_att_eff = N_att_phys * self.PHI_OVERLAP 
        S = self.D_std / N_att_eff 
        Gap = np.maximum(0, S - (2 * R_sens))
        if V_ms <= 0.01: return 999 
        return (Gap / V_ms) / 60.0

    def solve_fleet_for_z(self, Target_Z, V_kts, R_sens):
        V_ms = self.knots_to_ms(V_kts)
        Gap_req = Target_Z * 60.0 * V_ms
        S_req = Gap_req + (2 * R_sens)
        if S_req <= 0: return 200
        N_att_eff_req = self.D_std / S_req
        N_att_phys_req = N_att_eff_req / self.PHI_OVERLAP
        return N_att_phys_req * self.get_fto_vehicle()

    # --- CALCOLO SISTEMA COMPLETO ---
    def calculate_system_complete(self, L_total_km, N_tot_uuv_per_module, n_days, cost_uuv_k, cost_batt, t_charge):
        
        n_modules = np.ceil((L_total_km * 1000.0) / self.D_std)
        if n_modules < 1: n_modules = 1
        
        FTO_v = self.get_fto_vehicle()
        FTO_b = self.get_fto_battery(t_charge)
        N_att_per_mod = N_tot_uuv_per_module / FTO_v
        N_tot_batt_per_mod = max(N_tot_uuv_per_module, np.ceil(N_att_per_mod * FTO_b))
        N_rack_per_mod = max(0, N_tot_batt_per_mod - N_tot_uuv_per_module)
        
        E_mission_kWh = self.P_drone_avg * self.T_miss
        P_slot_kW = E_mission_kWh / t_charge
        Power_Peak_SBS_kW = (N_rack_per_mod * P_slot_kW) + self.P_staz_hotel
        Power_Total_Sys_MW = (Power_Peak_SBS_kW * n_modules) / 1000.0
        
        t_idle = max(0, self.T_miss - t_charge)
        cycle_efficiency = "BILANCIATO"
        if t_idle > self.T_miss * 0.5: cycle_efficiency = "AGGRESSIVO"
        if t_charge > self.T_miss: cycle_efficiency = "LENTO"

        Data_req_SBS_Mbps = (N_att_per_mod + self.N_int_safety) * self.BW_stream
        Data_Total_Sys_Gbps = (Data_req_SBS_Mbps * n_modules) / 1000.0
        
        # Breakdown Costi
        c_fleet_mod = N_tot_uuv_per_module * cost_uuv_k * 1000.0
        c_batt_mod = N_tot_batt_per_mod * cost_batt
        c_infra_base_mod = (N_rack_per_mod * self.C_chg) + (3 * self.C_srv) 
        
        N_rech_24h = N_att_per_mod * (24.0 / self.T_miss)
        E_daily_mod = (N_rech_24h * E_mission_kWh) + (self.P_staz_hotel * 24.0)
        c_energy_mod = E_daily_mod * n_days * self.C_kwh
        c_buoy_mod = self.C_BUOY_POPUP 
        
        Cost_Cable_km_Dynamic = self.C_cable_base + (self.C_cable_mw * Power_Total_Sys_MW)
        
        C_TOT_CABLE = (L_total_km * Cost_Cable_km_Dynamic * 1000.0) + (self.C_mob_vessel * 1000.0)
        C_TOT_FLEET = c_fleet_mod * n_modules
        C_TOT_BATT = c_batt_mod * n_modules
        # Voce Rinomata
        C_TOT_SYSTEM_INFRA = (c_infra_base_mod + c_energy_mod + c_buoy_mod) * n_modules
        
        CAPEX_Total = C_TOT_CABLE + C_TOT_FLEET + C_TOT_BATT + C_TOT_SYSTEM_INFRA
        CAPEX_per_km = CAPEX_Total / L_total_km
        
        cost_breakdown = {
            'Flotta': C_TOT_FLEET,
            'Batterie': C_TOT_BATT,
            'Cavi & Posa': C_TOT_CABLE,
            'Infrastruttura del sistema': C_TOT_SYSTEM_INFRA
        }
        
        return (CAPEX_per_km, int(N_tot_batt_per_mod), int(N_rack_per_mod), 
                Power_Peak_SBS_kW, Data_req_SBS_Mbps, cost_breakdown, cycle_efficiency, 
                Power_Total_Sys_MW, Data_Total_Sys_Gbps, Cost_Cable_km_Dynamic)

    def calculate_intervention_complex(self, z_blind, r_sens, v_interceptor_kts, t_c2, r_action, t_eval):
        v_m_min = self.knots_to_m_min(v_interceptor_kts)
        if v_m_min <= 0: return 999, 999, 0
        dist_run = np.maximum(0, r_sens - r_action)
        t_run = dist_run / v_m_min
        t_total = z_blind + t_c2 + t_run + t_eval
        time_saved = (r_sens/v_m_min) - t_run
        return t_total, t_run, time_saved

K = CUI_Kernel()

# ==============================================================================
# 2. INTERFACCIA DASH (V38)
# ==============================================================================
app = dash.Dash(__name__, title="CUI CONTROL ROOM V38")
server = app.server  # <--- RIGA NUOVA FONDAMENTALE PER IL CLOUD

slider_style = {'width': '23%', 'fontSize': '12px'}
slider_row_style = {'display': 'flex', 'justifyContent': 'space-between', 'padding': '5px', 'borderRadius': '8px', 'marginBottom': '8px'}
data_box_style = {'display': 'flex', 'flexDirection': 'column', 'alignItems': 'center', 'justifyContent': 'center', 'fontSize': '11px', 'lineHeight': '1.2', 'textAlign': 'center', 'width': '14%'}

app.layout = html.Div(style={'fontFamily': 'Segoe UI, sans-serif', 'backgroundColor': '#ffffff', 'height': '100vh', 'padding': '10px'}, children=[
    
    html.H3("CUI SWARM SHIELD - ANALYTICAL DASHBOARD", style={'textAlign': 'center', 'color': '#2c3e50', 'margin': '5px'}),
    
    # RIGA 1: INFRASTRUTTURA & DATI TECNICI
    html.Div(style={**slider_row_style, 'backgroundColor': '#e8f6f3', 'border': '1px solid #1abc9c'}, children=[
        html.Div(style={'width': '15%'}, children=[
            html.Label("🌍 Lunghezza (km)", style={'fontWeight': 'bold', 'color': '#16a085', 'fontSize': '12px'}),
            dcc.Slider(id='sld-len', min=10, max=200, step=10, value=20, marks={10:'10', 100:'100', 200:'200'}, tooltip={"placement": "bottom"})
        ]),
        # Box Dati Tecnici
        html.Div(id='box-infra', style={'width': '84%', 'display': 'flex', 'justifyContent': 'space-between'}, children="Calcolo...")
    ]),

    # RIGA 2: OPERATIVITÀ
    html.Div(style={**slider_row_style, 'backgroundColor': '#ecf0f1'}, children=[
        html.Div(style=slider_style, children=[
            html.Label("⏱️ Tempo Cieco (min)", style={'fontWeight': 'bold', 'color': '#e74c3c'}),
            dcc.Slider(id='sld-z', min=0, max=15, step=0.1, value=5, marks={0:'0', 5:'5', 10:'10', 15:'15'}, tooltip={"placement": "bottom"})
        ]),
        html.Div(style=slider_style, children=[
            html.Label("🚁 Flotta Modulo (N)", style={'fontWeight': 'bold', 'color': '#2980b9'}),
            dcc.Slider(id='sld-n', min=5, max=100, step=1, value=40, marks={10:'10', 50:'50', 100:'100'}, tooltip={"placement": "bottom"})
        ]),
        html.Div(style=slider_style, children=[
            html.Label("⚡ Vel. Pattuglia (kts)", style={'fontWeight': 'bold', 'color': '#2980b9'}),
            dcc.Slider(id='sld-v', min=0.5, max=2.0, step=0.1, value=1.5, marks={0.5:'0.5', 1.5:'1.5'}, tooltip={"placement": "bottom"})
        ]),
        html.Div(style=slider_style, children=[
            html.Label("📡 Portata Sensore (m)", style={'fontWeight': 'bold', 'color': '#27ae60'}),
            dcc.Slider(id='sld-r', min=1, max=100, step=1, value=45, marks={1:'1', 50:'50', 100:'100'}, tooltip={"placement": "bottom"})
        ]),
    ]),

    # RIGA 3: LOGISTICA
    html.Div(style={**slider_row_style, 'backgroundColor': '#fff7e6', 'border': '1px solid #f39c12'}, children=[
        html.Div(style=slider_style, children=[
            html.Label("🔋 Autonomia (gg)", style={'fontWeight': 'bold', 'color': '#8e44ad'}),
            dcc.Slider(id='sld-days', min=1, max=14, step=1, value=7, marks={1:'1', 7:'7'}, tooltip={"placement": "bottom"})
        ]),
        html.Div(style=slider_style, children=[
            html.Label("⏳ Tempo Ricarica (h)", style={'fontWeight': 'bold', 'color': '#d35400'}),
            dcc.Slider(id='sld-t-charge', min=0.5, max=4.0, step=0.1, value=1.5, marks={0.5:'Fast', 1.5:'Std', 4:'Eco'}, tooltip={"placement": "bottom"})
        ]),
        html.Div(style=slider_style, children=[
            html.Label("💶 Costo Veicolo (k€)", style={'fontWeight': 'bold', 'color': '#7f8c8d'}),
            dcc.Slider(id='sld-cost-uuv', min=5, max=40, step=1, value=15, marks={10:'10k', 30:'30k'}, tooltip={"placement": "bottom"})
        ]),
        html.Div(style=slider_style, children=[
            html.Label("🔋 Costo Batteria (€)", style={'fontWeight': 'bold', 'color': '#7f8c8d'}),
            dcc.Slider(id='sld-cost-batt', min=500, max=3000, step=100, value=2000, marks={500:'0.5k', 2000:'2k'}, tooltip={"placement": "bottom"})
        ]),
    ]),

    # RIGA 4: INTERDIZIONE
    html.Div(style={**slider_row_style, 'backgroundColor': '#ffeaea'}, children=[
        html.Div(style={'width': '19%', 'fontSize': '12px'}, children=[
            html.Label("🚀 Vel. Intercetto (kts)", style={'fontWeight': 'bold', 'color': '#c0392b'}),
            dcc.Slider(id='sld-v-sprint', min=5, max=30, step=1, value=15, marks={5:'5', 15:'15', 30:'30'}, tooltip={"placement": "bottom"})
        ]),
        html.Div(style={'width': '19%', 'fontSize': '12px'}, children=[
            html.Label("📏 Dist. intercetto (m)", style={'fontWeight': 'bold', 'color': '#8e44ad'}),
            dcc.Slider(id='sld-r-eff', min=0, max=100, step=5, value=10, marks={0:'0', 50:'50', 100:'100'}, tooltip={"placement": "bottom"})
        ]),
        html.Div(style={'width': '19%', 'fontSize': '12px'}, children=[
            html.Label("🧠 Tempo decisione (min)", style={'fontWeight': 'bold', 'color': '#c0392b'}),
            dcc.Slider(id='sld-t-c2', min=0, max=5, step=0.5, value=1, marks={0:'0', 2.5:'2.5', 5:'5'}, tooltip={"placement": "bottom"})
        ]),
        html.Div(style={'width': '19%', 'fontSize': '12px'}, children=[
            html.Label("👁️ Tempo valutazione (min)", style={'fontWeight': 'bold', 'color': '#d35400'}),
            dcc.Slider(id='sld-t-eval', min=0, max=10, step=0.5, value=2, marks={0:'0', 5:'5', 10:'10'}, tooltip={"placement": "bottom"})
        ]),
        html.Div(style={'width': '19%', 'fontSize': '12px'}, children=[
            html.Label("🎯 T. danneggiam. (min)", style={'fontWeight': 'bold', 'color': '#2c3e50'}),
            dcc.Slider(id='sld-t-target', min=5, max=60, step=1, value=20, marks={5:'5', 30:'30', 60:'60'}, tooltip={"placement": "bottom"})
        ]),
    ]),

    # AREA GRAFICI
    html.Div(style={'display': 'flex', 'height': '48vh', 'alignItems': 'center', 'justifyContent': 'space-between'}, children=[
        dcc.Graph(id='graph-3d', style={'width': '27%', 'height': '100%'}),
        dcc.Graph(id='graph-2d', style={'width': '24%', 'height': '100%'}),
        dcc.Graph(id='graph-capex', style={'width': '27%', 'height': '100%'}),
        dcc.Graph(id='graph-kpi', style={'width': '18%', 'height': '90%'}),
    ])
])

# ==============================================================================
# 3. CALLBACK
# ==============================================================================
@app.callback(
    [Output('sld-z', 'value'), Output('sld-n', 'value'), 
     Output('graph-3d', 'figure'), Output('graph-2d', 'figure'), 
     Output('graph-capex', 'figure'), Output('graph-kpi', 'figure'),
     Output('box-infra', 'children')],
    [Input('sld-len', 'value'),
     Input('sld-z', 'value'), Input('sld-n', 'value'), Input('sld-v', 'value'), Input('sld-r', 'value'),
     Input('sld-days', 'value'), Input('sld-t-charge', 'value'), 
     Input('sld-cost-uuv', 'value'), Input('sld-cost-batt', 'value'),
     Input('sld-v-sprint', 'value'), Input('sld-t-c2', 'value'), Input('sld-t-target', 'value'),
     Input('sld-r-eff', 'value'), Input('sld-t-eval', 'value')]
)
def update_simulation(len_total, z_in, n_in, v_in, r_in, days_in, t_charge_in, c_uuv_k_in, c_batt_in, v_sprint, t_c2, t_target, r_eff, t_eval):
    trigger = ctx.triggered_id
    calc_z, calc_n = z_in, n_in
    out_z, out_n = no_update, no_update
    
    # Anti-Loop
    if trigger == 'sld-z':
        new_n = K.solve_fleet_for_z(z_in, v_in, r_in)
        calc_n = new_n 
        if abs(new_n - n_in) > 0.5: out_n = new_n
    elif trigger in ['sld-n', 'sld-v', 'sld-r']:
        new_z = K.solve_blind_time(n_in, v_in, r_in)
        calc_z = new_z 
        if abs(new_z - z_in) > 0.01: out_z = new_z
    elif trigger is None:
        calc_z = K.solve_blind_time(n_in, v_in, r_in)
        out_z = calc_z

    # --- CALCOLO SISTEMA ---
    (curr_capex_km, n_batt_tot, n_batt_rack, power_req_sbs, data_req_sbs, 
     cost_breakdown, cycle_eff, power_tot_sys, data_tot_sys, cable_cost_km) = K.calculate_system_complete(
        len_total, calc_n, days_in, c_uuv_k_in, c_batt_in, t_charge_in
    )
    
    # Totale CAPEX M€
    total_capex_meuro = sum(cost_breakdown.values()) / 1000000.0
    sys_cost_k_km = curr_capex_km / 1000.0
    
    # Text Box
    box_text = [
        html.Div(style=data_box_style, children=[html.Span("Assorbimento massimo di una SBS"), html.B(f"{power_req_sbs:.1f} kW", style={'color': '#d35400'})]),
        html.Div(style=data_box_style, children=[html.Span("Assorbimento totale da rete a terra"), html.B(f"{power_tot_sys:.1f} MW", style={'color': '#c0392b'})]),
        html.Div(style=data_box_style, children=[html.Span("Traffico dati massimo di una SBS"), html.B(f"{data_req_sbs:.0f} Mbps", style={'color': '#2980b9'})]),
        html.Div(style=data_box_style, children=[html.Span("Traffico dati massimo previsto"), html.B(f"{data_tot_sys:.1f} Gbps", style={'color': '#8e44ad'})]),
        html.Div(style=data_box_style, children=[html.Span("Costo chilometrico cavo (€/Km)"), html.B(f"{cable_cost_km:.0f} k€", style={'color': '#27ae60'})]),
        html.Div(style=data_box_style, children=[html.Span("Costo chilometrico sistema (€/Km)"), html.B(f"{sys_cost_k_km:.0f} k€", style={'color': '#16a085'})]),
        html.Div(style=data_box_style, children=[html.Span("CAPEX Totale"), html.B(f"{total_capex_meuro:.1f} M€", style={'color': '#2c3e50', 'fontSize': '14px'})])
    ]

    # --- Interdizione ---
    t_total, t_run, t_saved = K.calculate_intervention_complex(calc_z, r_in, v_sprint, t_c2, r_eff, t_eval)
    is_success = t_total <= t_target
    status_icon = "✅" if is_success else "❌"
    status_color = "green" if is_success else "red"
    status_text = "SUCCESSO" if is_success else "FALLIMENTO"
    
    # --- 1. 3D HEATMAP COSTI ---
    N_range = np.linspace(5, 100, 30)
    V_range = np.linspace(0.5, 2.0, 30)
    X_n, Y_v = np.meshgrid(N_range, V_range)
    Z_plot = np.zeros_like(X_n)
    C_surf = np.zeros_like(X_n) 
    
    c_min_scenario, _, _, _, _, _, _, _, _, _ = K.calculate_system_complete(len_total, 5, 1, 5, 500, 4.0)
    c_max_scenario, _, _, _, _, _, _, _, _, _ = K.calculate_system_complete(len_total, 100, 14, 40, 3000, 0.5)

    for i in range(X_n.shape[0]):
        for j in range(X_n.shape[1]):
            val_z = K.solve_blind_time(X_n[i,j], Y_v[i,j], r_in)
            Z_plot[i,j] = 20.0 - min(val_z, 20.0) 
            c_km_pt, _, _, _, _, _, _, _, _, _ = K.calculate_system_complete(
                len_total, X_n[i,j], days_in, c_uuv_k_in, c_batt_in, t_charge_in
            )
            C_surf[i,j] = c_km_pt
            
    fig3d = go.Figure()
    fig3d.add_trace(go.Surface(
        z=Z_plot, x=X_n, y=Y_v, 
        surfacecolor=C_surf,
        colorscale='RdYlGn_r', cmin=c_min_scenario, cmax=c_max_scenario, 
        opacity=0.9, 
        contours={"x": {"show": True, "color":"lightgrey"}, "y": {"show": True, "color":"lightgrey"}, "z": {"show": False}}, 
        hovertemplate="Vel: %{y:.1f} kts<br>Flotta: %{x:.0f}<br>Z: %{z:.2f} min<br>CAPEX: %{surfacecolor:.0f} €/km<extra></extra>", 
        showscale=False
    ))

    theta = np.linspace(0, 2*np.pi, 60)
    el_n = 40 + 20 * np.cos(theta)
    el_v = 1.15 + 0.35 * np.sin(theta)
    el_z_real = np.array([K.solve_blind_time(n, v, r_in) for n, v in zip(el_n, el_v)])
    el_z_plot = 20.0 - np.minimum(el_z_real, 20.0)
    fig3d.add_trace(go.Scatter3d(x=el_n, y=el_v, z=el_z_plot, mode='lines', line=dict(color='blue', width=6), name='Zona 20-60'))
    
    pt_z_plot = 20.0 - min(calc_z, 20.0)
    fig3d.add_trace(go.Scatter3d(x=[calc_n], y=[v_in], z=[pt_z_plot], mode='markers', marker=dict(size=6, color='red', line=dict(color='black', width=1)), name='Attuale', hoverinfo='skip'))

    annotations = [dict(
        x=calc_n, y=v_in, z=pt_z_plot,
        text=(
            f"<b>Combinazione Attuale</b><br>"
            f"Velocità: {v_in} kts<br>"
            f"Flotta: {int(calc_n)}<br>"
            f"Tempo Cieco: {calc_z:.2f} min<br>"
            f"Costo/km: {sys_cost_k_km:.0f} k€<br>"
            f"<span style='color:{status_color}'><b>{status_icon} {status_text}</b></span>"
        ),
        showarrow=True, arrowhead=1, arrowsize=1, arrowwidth=2,
        ax=-100, ay=-80, bgcolor="white", bordercolor="red", borderwidth=3, opacity=0.9, font=dict(color="black", size=11), align="left"
    )]

    scene_config = dict(
        xaxis=dict(title='Flotta (N)', showbackground=True, backgroundcolor='#f4f6f7', gridcolor='#bdc3c7', range=[5, 100], autorange=False),
        yaxis=dict(title='Velocità (kts)', showbackground=True, backgroundcolor='#f4f6f7', gridcolor='#bdc3c7', range=[0.5, 2.0], autorange=False),
        zaxis=dict(title='Tempo Cieco (min)', showbackground=True, backgroundcolor='#f4f6f7', gridcolor='#bdc3c7', range=[0, 20], autorange=False, tickmode='array', tickvals=[0,10,20], ticktext=["20","10","0"]),
        annotations=annotations
    )
    if trigger is None: scene_config['camera'] = dict(projection=dict(type="orthographic"), eye=dict(x=-1.5, y=-1.5, z=1.2))

    fig3d.update_layout(title="<b>TRADE-OFF</b>", uirevision='TheRock', scene=scene_config, margin=dict(l=5, r=5, t=30, b=5), 
                        showlegend=True, legend=dict(x=0, y=1, orientation="h", bgcolor='rgba(255,255,255,0.5)'))

    # --- 2. 2D (AUTOSCALE + LEGENDA) ---
    range_r = np.linspace(1, 100, 100)
    z_curve = [K.solve_blind_time(calc_n, v_in, r) for r in range_r]
    fig2d = go.Figure()
    fig2d.add_trace(go.Scatter(x=range_r, y=z_curve, mode='lines', line=dict(color='#2980b9', width=3), name='Curva portata/tempo cieco'))
    fig2d.add_trace(go.Scatter(x=[r_in], y=[calc_z], mode='markers+text', marker=dict(size=10, color='red'), text=[f"{r_in}m"], textposition="top center", name='Portata Attuale'))
    
    fig2d.update_layout(title="<b>PORTATA</b>", xaxis_title='Range (m)', yaxis_title='Z (min)', plot_bgcolor="white", margin=dict(l=30, r=10, t=30, b=30),
                        showlegend=True, legend=dict(x=0.5, y=1.1, orientation="h", xanchor="center"))
    fig2d.update_yaxes(autorange="reversed")

    # --- 3. CAPEX ---
    labels = list(cost_breakdown.keys())
    values = [v / 1000000.0 for v in cost_breakdown.values()] 
    colors = ['#3498db', '#9b59b6', '#34495e', '#f1c40f', '#e74c3c']
    
    fig_capex = go.Figure(go.Bar(x=labels, y=values, marker_color=colors, text=[f"{v:.1f} M€" for v in values], textposition='auto'))
    fig_capex.update_layout(title="<b>CAPEX (M€)</b>", yaxis=dict(title='M€', showgrid=True, gridcolor='lightgrey'), xaxis=dict(showgrid=False), plot_bgcolor='white', margin=dict(l=30, r=10, t=30, b=30))

    # --- 4. KPI ---
    fig_kpi = go.Figure(go.Bar(x=['Tot', 'Buffer'], y=[n_batt_tot, n_batt_rack], marker_color=['#2ecc71', '#f39c12'], text=[n_batt_tot, n_batt_rack], textposition='auto'))
    fig_kpi.update_layout(title="<b>BATTERIE (Mod)</b>", yaxis=dict(showgrid=False), xaxis=dict(showgrid=False), plot_bgcolor='white', margin=dict(l=20, r=20, t=30, b=20))

    return out_z, out_n, fig3d, fig2d, fig_capex, fig_kpi, box_text

if __name__ == '__main__':
    app.run(debug=True, use_reloader=False)