# -*- coding: utf-8 -*-
import dash
from dash import dcc, html, Input, Output, ctx, no_update
import plotly.graph_objects as go
import numpy as np
import traceback

# ==============================================================================
# 1A. KERNEL MATEMATICO LOGISTICA (V38)
# ==============================================================================
class CUI_Kernel:
    def __init__(self):
        self.D_std = 8900.0      
        self.T_miss = 3.0        
        self.T_swap = 0.1        
        self.P_drone_avg = 0.5   
        self.P_staz_hotel = 3.0  
        self.BW_stream = 20.0    
        self.N_int_safety = 2    
        self.C_chg = 5000        
        self.C_kwh = 400         
        self.C_srv = 8000        
        self.C_cable_base = 150  
        self.C_cable_mw = 100    
        self.C_mob_vessel = 500  
        self.C_BUOY_POPUP = 25000 
        self.PHI_OVERLAP = 2.0 

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
        
        Data_req_SBS_Mbps = (N_att_per_mod + self.N_int_safety) * self.BW_stream
        Data_Total_Sys_Gbps = (Data_req_SBS_Mbps * n_modules) / 1000.0
        
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
        C_TOT_SYSTEM_INFRA = (c_infra_base_mod + c_energy_mod + c_buoy_mod) * n_modules
        
        CAPEX_Total = C_TOT_CABLE + C_TOT_FLEET + C_TOT_BATT + C_TOT_SYSTEM_INFRA
        CAPEX_per_km = CAPEX_Total / L_total_km
        
        cost_breakdown = {'Flota': C_TOT_FLEET, 'Baterías': C_TOT_BATT, 'Cables y Tendido': C_TOT_CABLE, 'Infraestructura del Sistema': C_TOT_SYSTEM_INFRA}
        return (CAPEX_per_km, int(N_tot_batt_per_mod), int(N_rack_per_mod), Power_Peak_SBS_kW, Data_req_SBS_Mbps, cost_breakdown, "BALANCEADO", Power_Total_Sys_MW, Data_Total_Sys_Gbps, Cost_Cable_km_Dynamic)

    def calculate_intervention_complex(self, z_blind, r_sens, v_interceptor_kts, t_c2, r_action, t_eval):
        v_m_min = self.knots_to_m_min(v_interceptor_kts)
        if v_m_min <= 0: return 999, 999, 0
        dist_run = np.maximum(0, r_sens - r_action)
        t_run = dist_run / v_m_min
        t_total = z_blind + t_c2 + t_run + t_eval
        time_saved = (r_sens/v_m_min) - t_run
        return t_total, t_run, time_saved

K_log = CUI_Kernel() 

# ==============================================================================
# 1B. KERNEL MATEMATICO FISICA (V78)
# ==============================================================================
class CUI_Physics_Kernel:
    def __init__(self):
        self.FTO = 1.05

    def calculate_spacing(self, N_tot, L_km):
        if N_tot <= 0: return 99999.0
        N_active = N_tot / self.FTO
        if N_active < 1: return 99999.0
        return (L_km * 1000.0) / (N_active / 2.0)

    def get_R_eff(self, R_teo, k):
        if k <= 0.0: return R_teo
        return R_teo * (1.0 - (0.10 ** (1.0 / k)))

    def calculate_effective_chord(self, y_point, path_y, R_teo, k_factor):
        R_eff = self.get_R_eff(R_teo, k_factor)
        dist_perp = abs(y_point - path_y)
        if dist_perp >= R_eff: return 0.0
        try: x_max = np.sqrt(R_eff**2 - dist_perp**2)
        except: return 0.0
        x_vals = np.linspace(0, x_max, 30) 
        r_vals = np.sqrt(x_vals**2 + dist_perp**2)
        ratio = np.clip(r_vals / R_teo, 0, 1)
        weights = (1.0 - ratio) ** k_factor
        return (x_max * 2.0) * np.mean(weights)

    def calculate_probability_profile(self, y_range, N, L_km, R_teo, Offset, k_factor):
        S = self.calculate_spacing(N, L_km)
        if S <= 0: S = 99999.0
        P_vals = []
        for y in y_range:
            chord_n = self.calculate_effective_chord(y, Offset, R_teo, k_factor)
            chord_s = self.calculate_effective_chord(y, -Offset, R_teo, k_factor)
            p_tot = min(1.0, (chord_n + chord_s) / S)
            P_vals.append(p_tot * 100.0)
        return np.array(P_vals)

    def find_iso_distance(self, N, L_km, R_teo, Offset, Target_Prob, k_factor):
        R_eff = self.get_R_eff(R_teo, k_factor)
        y_scan = np.linspace(0, R_eff + Offset + 20, 100)
        probs = self.calculate_probability_profile(y_scan, N, L_km, R_teo, Offset, k_factor)
        if len(probs) > 0 and probs[0] < Target_Prob: return -1.0 
        for i in range(len(probs)-1):
            if probs[i] >= Target_Prob and probs[i+1] < Target_Prob: return y_scan[i]
        return 0.0

K_phys = CUI_Physics_Kernel()

# ==============================================================================
# 2. INIZIALIZZAZIONE APP E STILI UNIFICATI (+ CSS ESTREMO)
# ==============================================================================
app = dash.Dash(__name__, title="CUI MASTER SHIELD - DIGITAL TWIN")
server = app.server

# STILE DEI TOOLTIP SCHIACCIATO AL MINIMO POSSIBILE
app.index_string = '''
<!DOCTYPE html>
<html>
    <head>
        {%metas%}
        <title>{%title%}</title>
        {%favicon%}
        {%css%}
        <style>
            .rc-slider-tooltip-inner {
                background-color: #ffffff !important;
                color: #2c3e50 !important;
                font-size: 10px !important;
                padding: 1px 4px !important;
                min-width: 10px !important;
                min-height: 10px !important;
                line-height: 1.2 !important;
                border: 1px solid #bdc3c7 !important;
                box-shadow: 0px 1px 2px rgba(0,0,0,0.1) !important;
                border-radius: 3px !important;
            }
            .rc-slider-tooltip-placement-bottom .rc-slider-tooltip-arrow {
                border-bottom-color: #bdc3c7 !important;
            }
        </style>
    </head>
    <body>
        {%app_entry%}
        <footer>
            {%config%}
            {%scripts%}
            {%renderer%}
        </footer>
    </body>
</html>
'''

# STILI MINIMALISTI
sld_style = {'width': '19%', 'fontSize': '11px'}
row_style = {'display': 'flex', 'justifyContent': 'space-between', 'padding': '4px 10px', 'borderRadius': '8px', 'marginBottom': '6px', 'boxShadow': '0 1px 2px rgba(0,0,0,0.1)'}
box_style = {'display': 'flex', 'flexDirection': 'column', 'alignItems': 'center', 'justifyContent': 'center', 'fontSize': '11px', 'textAlign': 'center', 'padding': '5px', 'border': '1px solid #ecf0f1', 'borderRadius': '5px', 'marginBottom': '2px', 'backgroundColor': '#f9f9f9'}
CAM_3D = dict(eye=dict(x=-1.5, y=-1.5, z=1.2))

# ==============================================================================
# 3. LAYOUT SUPREMO (SPAGNOLO)
# ==============================================================================
app.layout = html.Div(style={'fontFamily': 'Segoe UI', 'backgroundColor': '#f4f6f7', 'height': '100vh', 'overflow': 'hidden', 'display': 'flex', 'flexDirection': 'column'}, children=[
    
    html.Div(style={'backgroundColor': '#2c3e50', 'padding': '5px'}, children=[
        html.H3("🛡️ CUI SWARM SHIELD - UNIFIED DIGITAL TWIN", style={'textAlign': 'center', 'color': 'white', 'margin': '0'})
    ]),

    # --- PLANCIA DI COMANDO MINIMALISTA ---
    html.Div(style={'flex': '0 0 auto', 'padding': '5px'}, children=[
        
        # Riga 1
        html.Div(style={**row_style, 'backgroundColor': '#e8f6f3', 'border': '1px solid #1abc9c'}, children=[
            html.Div(style=sld_style, children=[html.Div(id='lbl-len', style={'fontWeight': 'bold', 'color': '#16a085', 'marginBottom':'2px'}), dcc.Slider(id='sld-len', min=np.log10(1), max=np.log10(200), step=0.01, value=np.log10(20), marks={np.log10(1):'1', np.log10(5):'5', np.log10(10):'10', np.log10(50):'50', np.log10(100):'100', np.log10(200):'200'})]),
            html.Div(style=sld_style, children=[html.Div(id='lbl-r', style={'fontWeight': 'bold', 'color': '#27ae60', 'marginBottom':'2px'}), dcc.Slider(id='sld-r', min=10, max=100, step=5, value=45, marks={10:'10', 40:'40', 70:'70', 100:'100'}, tooltip={"placement": "bottom", "always_visible": True})]),
            html.Div(style=sld_style, children=[html.Div(id='lbl-off', style={'fontWeight': 'bold', 'color': '#8e44ad', 'marginBottom':'2px'}), dcc.Slider(id='sld-off', min=0, max=100, step=5, value=15, marks={0:'0', 25:'25', 50:'50', 75:'75', 100:'100'}, tooltip={"placement": "bottom", "always_visible": True})]),
            html.Div(style=sld_style, children=[html.Div(id='lbl-k', style={'fontWeight': 'bold', 'color': '#16a085', 'marginBottom':'2px'}), dcc.Slider(id='sld-k', min=0.0, max=4.0, step=0.1, value=2.0, marks={0:'0', 1:'1', 2:'2', 3:'3', 4:'4'}, tooltip={"placement": "bottom", "always_visible": True})]),
            html.Div(style=sld_style, children=[html.Div(id='lbl-z', style={'fontWeight': 'bold', 'color': '#e74c3c', 'marginBottom':'2px'}), dcc.Slider(id='sld-z', min=0, max=15, step=0.1, value=5, marks={0:'0', 5:'5', 10:'10', 15:'15'}, tooltip={"placement": "bottom", "always_visible": True})]),
        ]),
        
        # Riga 2
        html.Div(style={**row_style, 'backgroundColor': '#ecf0f1', 'border': '1px solid #bdc3c7'}, children=[
            html.Div(style=sld_style, children=[html.Div(id='lbl-n', style={'fontWeight': 'bold', 'color': '#2980b9', 'marginBottom':'2px'}), dcc.Slider(id='sld-n', min=5, max=150, step=1, value=40, marks={5:'5', 50:'50', 100:'100', 150:'150'}, tooltip={"placement": "bottom", "always_visible": True})]),
            html.Div(style=sld_style, children=[html.Div(id='lbl-v', style={'fontWeight': 'bold', 'color': '#2980b9', 'marginBottom':'2px'}), dcc.Slider(id='sld-v', min=0.5, max=2.0, step=0.1, value=1.5, marks={0.5:'0.5', 1:'1', 1.5:'1.5', 2:'2.0'}, tooltip={"placement": "bottom", "always_visible": True})]),
            html.Div(style=sld_style, children=[html.Div(id='lbl-v-sprint', style={'fontWeight': 'bold', 'color': '#c0392b', 'marginBottom':'2px'}), dcc.Slider(id='sld-v-sprint', min=5, max=30, step=1, value=15, marks={5:'5', 15:'15', 30:'30'}, tooltip={"placement": "bottom", "always_visible": True})]),
            html.Div(style=sld_style, children=[
                html.Div(style={'display':'flex', 'width': '100%', 'justifyContent': 'space-between', 'marginBottom':'2px'}, children=[
                    html.Div(id='lbl-t-c2', style={'fontWeight': 'bold', 'color': '#d35400', 'fontSize': '10px'}),
                    html.Div(id='lbl-t-eval', style={'fontWeight': 'bold', 'color': '#d35400', 'fontSize': '10px'})
                ]),
                html.Div(style={'display':'flex', 'width': '100%'}, children=[
                    html.Div(style={'width': '50%', 'paddingRight': '2px'}, children=[dcc.Slider(id='sld-t-c2', min=0, max=5, step=0.5, value=1, marks={0:'0', 2.5:'2.5', 5:'5'}, tooltip={"placement": "bottom", "always_visible": True})]), 
                    html.Div(style={'width': '50%', 'paddingLeft': '2px'}, children=[dcc.Slider(id='sld-t-eval', min=0, max=10, step=0.5, value=2, marks={0:'0', 5:'5', 10:'10'}, tooltip={"placement": "bottom", "always_visible": True})])
                ])
            ]),
            html.Div(style=sld_style, children=[html.Div(id='lbl-t-target', style={'fontWeight': 'bold', 'color': '#2c3e50', 'marginBottom':'2px'}), dcc.Slider(id='sld-t-target', min=5, max=60, step=1, value=20, marks={5:'5', 30:'30', 60:'60'}, tooltip={"placement": "bottom", "always_visible": True})]),
        ]),
        
        # Riga 3
        html.Div(style={**row_style, 'backgroundColor': '#fff7e6', 'border': '1px solid #f39c12'}, children=[
            html.Div(style=sld_style, children=[html.Div(id='lbl-days', style={'fontWeight': 'bold', 'color': '#8e44ad', 'marginBottom':'2px'}), dcc.Slider(id='sld-days', min=1, max=14, step=1, value=7, marks={1:'1', 7:'7', 14:'14'}, tooltip={"placement": "bottom", "always_visible": True})]),
            html.Div(style=sld_style, children=[html.Div(id='lbl-t-charge', style={'fontWeight': 'bold', 'color': '#d35400', 'marginBottom':'2px'}), dcc.Slider(id='sld-t-charge', min=0.5, max=4.0, step=0.1, value=1.5, marks={0.5:'0.5', 2:'2', 4:'4'}, tooltip={"placement": "bottom", "always_visible": True})]),
            html.Div(style=sld_style, children=[html.Div(id='lbl-cost-uuv', style={'fontWeight': 'bold', 'color': '#7f8c8d', 'marginBottom':'2px'}), dcc.Slider(id='sld-cost-uuv', min=5, max=40, step=1, value=15, marks={5:'5', 20:'20', 40:'40'}, tooltip={"placement": "bottom", "always_visible": True})]),
            html.Div(style=sld_style, children=[html.Div(id='lbl-cost-batt', style={'fontWeight': 'bold', 'color': '#7f8c8d', 'marginBottom':'2px'}), dcc.Slider(id='sld-cost-batt', min=500, max=3000, step=100, value=2000, marks={500:'500', 1500:'1.5k', 3000:'3k'}, tooltip={"placement": "bottom", "always_visible": True})]),
            html.Div(style=sld_style, children=[html.Label(id='lbl-eff-range', style={'fontWeight': 'bold', 'color': '#16a085', 'marginTop':'12px'})]),
        ]),
    ]),

    # --- GRIGLIA GRAFICI ---
    html.Div(style={'flex': '1 1 auto', 'padding': '2px 10px', 'display': 'flex', 'flexDirection': 'column', 'justifyContent': 'space-between'}, children=[
        html.Div(style={'display': 'flex', 'justifyContent': 'space-between', 'height': '49%'}, children=[
            html.Div(style={'width': '35%', 'backgroundColor': 'white', 'borderRadius': '8px', 'boxShadow': '0 1px 3px rgba(0,0,0,0.1)'}, children=[dcc.Graph(id='g-3d-trade', style={'height': '100%'})]),
            html.Div(style={'width': '32%', 'backgroundColor': 'white', 'borderRadius': '8px', 'boxShadow': '0 1px 3px rgba(0,0,0,0.1)'}, children=[dcc.Graph(id='g-3d-safe', style={'height': '100%'})]),
            html.Div(style={'width': '32%', 'backgroundColor': 'white', 'borderRadius': '8px', 'boxShadow': '0 1px 3px rgba(0,0,0,0.1)'}, children=[dcc.Graph(id='g-3d-warn', style={'height': '100%'})]),
        ]),
        html.Div(style={'display': 'flex', 'justifyContent': 'space-between', 'height': '49%', 'marginTop': '5px'}, children=[
            html.Div(id='box-infra', style={'width': '12%', 'display': 'flex', 'flexDirection': 'column', 'justifyContent': 'space-evenly', 'backgroundColor': 'white', 'padding': '5px', 'borderRadius': '8px'}),
            html.Div(style={'width': '38%', 'backgroundColor': 'white', 'borderRadius': '8px', 'boxShadow': '0 1px 3px rgba(0,0,0,0.1)'}, children=[dcc.Graph(id='g-map', style={'height': '100%'})]),
            html.Div(style={'width': '18%', 'backgroundColor': 'white', 'borderRadius': '8px', 'boxShadow': '0 1px 3px rgba(0,0,0,0.1)'}, children=[dcc.Graph(id='g-range', style={'height': '100%'})]),
            html.Div(style={'width': '18%', 'backgroundColor': 'white', 'borderRadius': '8px', 'boxShadow': '0 1px 3px rgba(0,0,0,0.1)'}, children=[dcc.Graph(id='g-capex', style={'height': '100%'})]),
            html.Div(style={'width': '12%', 'backgroundColor': 'white', 'borderRadius': '8px', 'boxShadow': '0 1px 3px rgba(0,0,0,0.1)'}, children=[dcc.Graph(id='g-batt', style={'height': '100%'})]),
        ]),
    ])
])

# ==============================================================================
# 4. IL CERVELLO UNIFICATO (THE MASTER CALLBACK)
# ==============================================================================
@app.callback(
    [Output('sld-z', 'value'), Output('sld-n', 'value'), Output('lbl-eff-range', 'children'),
     Output('lbl-len', 'children'), Output('lbl-r', 'children'), Output('lbl-off', 'children'),
     Output('lbl-k', 'children'), Output('lbl-z', 'children'), Output('lbl-n', 'children'),
     Output('lbl-v', 'children'), Output('lbl-v-sprint', 'children'), Output('lbl-t-c2', 'children'),
     Output('lbl-t-eval', 'children'), Output('lbl-t-target', 'children'), Output('lbl-days', 'children'),
     Output('lbl-t-charge', 'children'), Output('lbl-cost-uuv', 'children'), Output('lbl-cost-batt', 'children'),
     Output('g-3d-trade', 'figure'), Output('g-3d-safe', 'figure'), Output('g-3d-warn', 'figure'),
     Output('g-map', 'figure'), Output('g-range', 'figure'), Output('g-capex', 'figure'), 
     Output('g-batt', 'figure'), Output('box-infra', 'children')],
    [Input('sld-len', 'value'), Input('sld-z', 'value'), Input('sld-n', 'value'), 
     Input('sld-v', 'value'), Input('sld-r', 'value'), Input('sld-off', 'value'), Input('sld-k', 'value'),
     Input('sld-days', 'value'), Input('sld-t-charge', 'value'), Input('sld-cost-uuv', 'value'), 
     Input('sld-cost-batt', 'value'), Input('sld-v-sprint', 'value'), Input('sld-t-c2', 'value'), 
     Input('sld-t-eval', 'value'), Input('sld-t-target', 'value')]
)
def update_master(l_km_exp, z_in, n_in, v_in, r_in, off_in, k_in, days, t_chg, c_uuv, c_batt, v_spr, t_c2, t_eval, t_tgt):
    trigger = ctx.triggered_id
    calc_z, calc_n = z_in, n_in
    out_z, out_n = no_update, no_update
    
    # Decodifica la scala logaritmica in chilometri lineari
    l_km = 10 ** l_km_exp
    
    # Anti-Loop Logistica
    if trigger == 'sld-z':
        new_n = K_log.solve_fleet_for_z(z_in, v_in, r_in)
        calc_n = new_n 
        if abs(new_n - n_in) > 0.5: out_n = new_n
    elif trigger in ['sld-n', 'sld-v', 'sld-r']:
        new_z = K_log.solve_blind_time(n_in, v_in, r_in)
        calc_z = new_z 
        if abs(new_z - z_in) > 0.01: out_z = new_z
    elif trigger is None:
        calc_z = K_log.solve_blind_time(n_in, v_in, r_in)
        out_z = calc_z

    # Generazione Testi Dinamici per le Etichette dei Cursori (TRADOTTE)
    l_len = f"🌍 Longitud del Cable: {l_km:.1f} km"
    l_r = f"📡 Alcance: {r_in} m"
    l_off = f"↔️ Offset: {off_in} m"
    l_k = f"📡 Factor K: {k_in:.1f}"
    l_z = f"⏱️ Tiempo Ciego: {calc_z:.1f} min"
    l_n = f"🚁 Flota (N): {int(calc_n)}"
    l_v = f"⚡ V. Patrulla: {v_in:.1f} nudos"
    l_vspr = f"🚀 V. Intercepción: {v_spr} nudos"
    l_tc2 = f"🧠 C2: {t_c2:.1f} min"
    l_teval = f"👁️ Eval: {t_eval:.1f} min"
    l_ttgt = f"🎯 T. Daño: {t_tgt} min"
    l_days = f"🔋 Autonomía: {days} días"
    l_tchg = f"⏳ T. Recarga: {t_chg:.1f} h"
    l_cuuv = f"💶 C. UUV: {c_uuv} k€"
    l_cbatt = f"🔋 C. Bat: {c_batt} €"

    # Calcoli Finanziari
    (capex_km, n_batt_tot, n_batt_rack, p_sbs, d_sbs, c_brk, _, p_tot, d_tot, c_cbl) = K_log.calculate_system_complete(l_km, calc_n, days, c_uuv, c_batt, t_chg)
    tot_capex = sum(c_brk.values()) / 1e6
    sys_c_km = capex_km / 1e3
    
    t_tot_int, t_run, _ = K_log.calculate_intervention_complex(calc_z, r_in, v_spr, t_c2, off_in, t_eval)
    is_success = t_tot_int <= t_tgt

    box_text = [
        html.Div(style=box_style, children=[html.Span("Red Terrestre"), html.B(f"{p_tot:.1f} MW", style={'color': '#c0392b'})]),
        html.Div(style=box_style, children=[html.Span("Tráfico Máx"), html.B(f"{d_tot:.1f} Gbps", style={'color': '#8e44ad'})]),
        html.Div(style=box_style, children=[html.Span("Costo del Sistema"), html.B(f"{sys_c_km:.0f} k€/km", style={'color': '#16a085'})]),
        html.Div(style=box_style, children=[html.Span("CAPEX Total"), html.B(f"{tot_capex:.1f} M€", style={'color': '#2c3e50', 'fontSize': '14px'})])
    ]

    r_eff = K_phys.get_R_eff(r_in, float(k_in))
    lbl_r_eff = f"🎯 Alcance Útil >10%: {r_eff:.1f} m"

    status_icon = "✅" if is_success else "❌"
    status_color = "green" if is_success else "red"
    status_text = "ÉXITO" if is_success else "FRACASO"

    # Grafici 3D
    N_rng = np.linspace(5, 100, 20)
    V_rng = np.linspace(0.5, 2.0, 20)
    X_n, Y_v = np.meshgrid(N_rng, V_rng)
    Z_plot = np.zeros_like(X_n)
    C_surf = np.zeros_like(X_n) 
    for i in range(X_n.shape[0]):
        for j in range(X_n.shape[1]):
            val_z = K_log.solve_blind_time(X_n[i,j], Y_v[i,j], r_in)
            Z_plot[i,j] = 20.0 - min(val_z, 20.0) 
            c_km_pt, _, _, _, _, _, _, _, _, _ = K_log.calculate_system_complete(l_km, X_n[i,j], days, c_uuv, c_batt, t_chg)
            C_surf[i,j] = c_km_pt
    fig_trade = go.Figure(go.Surface(z=Z_plot, x=X_n, y=Y_v, surfacecolor=C_surf, colorscale='RdYlGn_r', showscale=False))
    pt_z_plot = 20.0 - min(calc_z, 20.0)
    fig_trade.add_trace(go.Scatter3d(x=[calc_n], y=[v_in], z=[pt_z_plot], mode='markers', marker=dict(size=6, color='red', line=dict(color='black', width=1))))
    
    annotations = [dict(
        x=calc_n, y=v_in, z=pt_z_plot,
        text=(f"<b>Combinación Actual</b><br>Velocidad: {v_in} nudos<br>Flota: {int(calc_n)}<br>Tiempo Ciego: {calc_z:.2f} min<br>Costo/km: {sys_c_km:.0f} k€<br><span style='color:{status_color}'><b>{status_icon} {status_text}</b></span>"),
        showarrow=True, arrowhead=1, arrowsize=1, arrowwidth=2, ax=-100, ay=-80, bgcolor="white", bordercolor="red", borderwidth=3, opacity=0.9, font=dict(color="black", size=11), align="left"
    )]
    fig_trade.update_layout(title="<b>TRADE-OFF CAPEX</b>", scene=dict(camera=CAM_3D, xaxis_title='Flota (N)', yaxis_title='Velocidad (nudos)', zaxis_title='Tiempo Ciego (min)', annotations=annotations), margin=dict(l=0, r=0, t=30, b=0))

    Off_rng = np.linspace(0, 100, 20)
    X_off, Y_n_phys = np.meshgrid(Off_rng, N_rng)
    Z_90, Z_10 = np.zeros_like(X_off), np.zeros_like(X_off)
    for i in range(len(N_rng)):
        for j in range(len(Off_rng)):
            Z_90[i,j] = max(0, K_phys.find_iso_distance(N_rng[i], l_km, r_in, Off_rng[j], 90.0, float(k_in)))
            Z_10[i,j] = K_phys.find_iso_distance(N_rng[i], l_km, r_in, Off_rng[j], 10.0, float(k_in))

    curr90 = K_phys.find_iso_distance(calc_n, l_km, r_in, off_in, 90.0, float(k_in))
    fig_90 = go.Figure(go.Surface(z=Z_90, x=X_off, y=Y_n_phys, colorscale='Reds', showscale=False))
    fig_90.add_trace(go.Scatter3d(x=[off_in], y=[calc_n], z=[max(0, curr90)], mode='markers', marker=dict(size=6, color='yellow')))
    fig_90.update_layout(title="<b>ZONA SEGURA (>90%)</b>", scene=dict(camera=CAM_3D, xaxis_title="Offset", yaxis_title="Flota (N)", zaxis_title="Distancia de Detección (m)"), margin=dict(l=0, r=0, t=30, b=0))

    curr10 = K_phys.find_iso_distance(calc_n, l_km, r_in, off_in, 10.0, float(k_in))
    fig_10 = go.Figure(go.Surface(z=Z_10, x=X_off, y=Y_n_phys, colorscale='Blues', showscale=False))
    fig_10.add_trace(go.Scatter3d(x=[off_in], y=[calc_n], z=[curr10], mode='markers', marker=dict(size=6, color='cyan')))
    fig_10.update_layout(title="<b>ZONA DE ADVERTENCIA (>10%)</b>", scene=dict(camera=CAM_3D, xaxis_title="Offset", yaxis_title="Flota (N)", zaxis_title="Distancia de Detección (m)"), margin=dict(l=0, r=0, t=30, b=0))

    # Grafici 2D
    Y_max = r_in + off_in + 10
    Y_arr = np.linspace(-Y_max, Y_max, 100)
    P_arr = K_phys.calculate_probability_profile(np.abs(Y_arr), calc_n, l_km, r_in, off_in, float(k_in))
    fig_map = go.Figure(data=go.Heatmap(z=np.column_stack((P_arr, P_arr)), x=[0, 100], y=Y_arr, colorscale=[[0, '#000'], [0.05, '#00F'], [0.1, '#0FF'], [0.4, '#0F0'], [0.8, '#FF0'], [1, '#F00']], zmin=0, zmax=100))
    fig_map.add_shape(type="line", x0=0, x1=100, y0=0, y1=0, line=dict(color="lime", width=2))
    fig_map.add_shape(type="line", x0=0, x1=100, y0=off_in, y1=off_in, line=dict(color="white", width=1, dash="dash"))
    fig_map.add_shape(type="line", x0=0, x1=100, y0=-off_in, y1=-off_in, line=dict(color="white", width=1, dash="dash"))
    fig_map.update_layout(title=f"<b>PROBABILIDAD (R_eff={r_eff:.1f}m)</b>", yaxis_title="Distancia de Detección (m)", margin=dict(l=45, r=10, t=30, b=10), plot_bgcolor='#111', xaxis=dict(showticklabels=False))

    r_sweep = np.linspace(1, 100, 50)
    z_sweep = [K_log.solve_blind_time(calc_n, v_in, r) for r in r_sweep]
    fig_rng = go.Figure()
    fig_rng.add_trace(go.Scatter(x=r_sweep, y=z_sweep, mode='lines', line=dict(color='#2980b9')))
    fig_rng.add_trace(go.Scatter(x=[r_in], y=[calc_z], mode='markers', marker=dict(size=8, color='red')))
    fig_rng.update_layout(title="<b>TIEMPO CIECO vs ALCANCE</b>", xaxis_title="Alcance del Sensor (m)", yaxis_title="Tiempo Ciego (min)", margin=dict(l=45, r=10, t=30, b=35), showlegend=False)
    fig_rng.update_yaxes(autorange="reversed")

    lbls = ['Flota', 'Bat', 'Cables', 'Infra']
    vals = [c_brk['Flota']/1e6, c_brk['Baterías']/1e6, c_brk['Cables y Tendido']/1e6, c_brk['Infraestructura del Sistema']/1e6]
    fig_cap = go.Figure(go.Bar(x=lbls, y=vals, marker_color=['#3498db', '#9b59b6', '#34495e', '#f1c40f'], text=[f"{v:.1f}" for v in vals], textposition='auto'))
    fig_cap.update_layout(title="<b>CAPEX (M€)</b>", yaxis_title="Millones de €", margin=dict(l=40, r=10, t=30, b=20))

    fig_bat = go.Figure(go.Bar(x=['Tot', 'Buf'], y=[n_batt_tot, n_batt_rack], marker_color=['#2ecc71', '#f39c12'], text=[n_batt_tot, n_batt_rack], textposition='auto'))
    fig_bat.update_layout(title="<b>BATERÍAS</b>", yaxis_title="Unidades (N)", margin=dict(l=40, r=10, t=30, b=20))

    return out_z, out_n, lbl_r_eff, l_len, l_r, l_off, l_k, l_z, l_n, l_v, l_vspr, l_tc2, l_teval, l_ttgt, l_days, l_tchg, l_cuuv, l_cbatt, fig_trade, fig_90, fig_10, fig_map, fig_rng, fig_cap, fig_bat, box_text

if __name__ == '__main__':
    app.run(debug=True, use_reloader=False)