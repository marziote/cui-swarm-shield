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
        
        t_idle = max(0, self.T_miss - t_charge)
        cycle_efficiency = "BILANCIATO"
        if t_idle > self.T_miss * 0.5: cycle_efficiency = "AGGRESSIVO"
        if t_charge > self.T_miss: cycle_efficiency = "LENTO"

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
        
        cost_breakdown = {'Flotta': C_TOT_FLEET, 'Batterie': C_TOT_BATT, 'Cavi & Posa': C_TOT_CABLE, 'Infrastruttura del sistema': C_TOT_SYSTEM_INFRA}
        return (CAPEX_per_km, int(N_tot_batt_per_mod), int(N_rack_per_mod), Power_Peak_SBS_kW, Data_req_SBS_Mbps, cost_breakdown, cycle_efficiency, Power_Total_Sys_MW, Data_Total_Sys_Gbps, Cost_Cable_km_Dynamic)

    def calculate_intervention_complex(self, z_blind, r_sens, v_interceptor_kts, t_c2, r_action, t_eval):
        v_m_min = self.knots_to_m_min(v_interceptor_kts)
        if v_m_min <= 0: return 999, 999, 0
        dist_run = np.maximum(0, r_sens - r_action)
        t_run = dist_run / v_m_min
        t_total = z_blind + t_c2 + t_run + t_eval
        time_saved = (r_sens/v_m_min) - t_run
        return t_total, t_run, time_saved

K_log = CUI_Kernel() # MOTORE LOGISTICO

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
        N_side = N_active / 2.0
        return (L_km * 1000.0) / N_side

    def get_R_eff(self, R_teo, k):
        if k <= 0.0: return R_teo
        return R_teo * (1.0 - (0.10 ** (1.0 / k)))

    def calculate_effective_chord(self, y_point, path_y, R_teo, k_factor):
        R_eff = self.get_R_eff(R_teo, k_factor)
        dist_perp = abs(y_point - path_y)
        if dist_perp >= R_eff: return 0.0
        try:
            x_max = np.sqrt(R_eff**2 - dist_perp**2)
        except:
            return 0.0
        x_vals = np.linspace(0, x_max, 30) 
        r_vals = np.sqrt(x_vals**2 + dist_perp**2)
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
        R_eff = self.get_R_eff(R_teo, k_factor)
        y_scan = np.linspace(0, R_eff + Offset + 20, 100)
        probs = self.calculate_probability_profile(y_scan, N, L_km, R_teo, Offset, k_factor)
        if len(probs) > 0 and probs[0] < Target_Prob: return -1.0 
        for i in range(len(probs)-1):
            if probs[i] >= Target_Prob and probs[i+1] < Target_Prob:
                return y_scan[i]
        return 0.0

K_phys = CUI_Physics_Kernel() # MOTORE FISICO

# ==============================================================================
# 2. INIZIALIZZAZIONE APP MASTER E STILI
# ==============================================================================
app = dash.Dash(__name__, title="CUI MASTER SHIELD")
server = app.server

# Stili Logistica
slider_style_log = {'width': '23%', 'fontSize': '12px'}
slider_row_style_log = {'display': 'flex', 'justifyContent': 'space-between', 'padding': '5px', 'borderRadius': '8px', 'marginBottom': '8px'}
data_box_style_log = {'display': 'flex', 'flexDirection': 'column', 'alignItems': 'center', 'justifyContent': 'center', 'fontSize': '11px', 'lineHeight': '1.2', 'textAlign': 'center', 'width': '14%'}

# Stili Fisica
slider_style_phys = {'width': '23%', 'fontSize': '12px', 'padding': '5px'}
row_style_phys = {'display': 'flex', 'justifyContent': 'space-between', 'backgroundColor': 'white', 'borderRadius': '8px', 'padding': '10px', 'marginBottom': '10px', 'boxShadow': '0 2px 4px rgba(0,0,0,0.1)'}
CAM_3D = dict(eye=dict(x=-1.5, y=-1.5, z=1.2))

# ==============================================================================
# 3. LAYOUT MASTER A SCHEDE (TABS)
# ==============================================================================
app.layout = html.Div(style={'fontFamily': 'Segoe UI', 'backgroundColor': '#f4f6f7', 'minHeight': '100vh'}, children=[
    
    html.Div(style={'backgroundColor': '#2c3e50', 'padding': '15px'}, children=[
        html.H2("🛡️ CUI SWARM SHIELD - DIGITAL TWIN MASTER", style={'textAlign': 'center', 'color': 'white', 'margin': '0'})
    ]),

    dcc.Tabs(style={'height': '40px'}, children=[

        # ----------------------------------------------------------------------
        # TAB 1: LOGISTICA & CAPEX
        # ----------------------------------------------------------------------
        dcc.Tab(label='📊 ANALISI LOGISTICA, TRAFFICO & CAPEX', style={'fontWeight': 'bold'}, selected_style={'fontWeight': 'bold', 'borderTop': '4px solid #3498db'}, children=[
            
            html.Div(style={'backgroundColor': '#ffffff', 'minHeight': '85vh', 'padding': '10px'}, children=[
                
                # RIGA 1: INFRASTRUTTURA & DATI TECNICI
                html.Div(style={**slider_row_style_log, 'backgroundColor': '#e8f6f3', 'border': '1px solid #1abc9c'}, children=[
                    html.Div(style={'width': '15%'}, children=[
                        html.Label("🌍 Lunghezza (km)", style={'fontWeight': 'bold', 'color': '#16a085', 'fontSize': '12px'}),
                        dcc.Slider(id='sld-len', min=10, max=200, step=10, value=20, marks={10:'10', 100:'100', 200:'200'}, tooltip={"placement": "bottom"})
                    ]),
                    html.Div(id='box-infra', style={'width': '84%', 'display': 'flex', 'justifyContent': 'space-between'}, children="Calcolo...")
                ]),

                # RIGA 2: OPERATIVITÀ
                html.Div(style={**slider_row_style_log, 'backgroundColor': '#ecf0f1'}, children=[
                    html.Div(style=slider_style_log, children=[
                        html.Label("⏱️ Tempo Cieco (min)", style={'fontWeight': 'bold', 'color': '#e74c3c'}),
                        dcc.Slider(id='sld-z', min=0, max=15, step=0.1, value=5, marks={0:'0', 5:'5', 10:'10', 15:'15'}, tooltip={"placement": "bottom"})
                    ]),
                    html.Div(style=slider_style_log, children=[
                        html.Label("🚁 Flotta Modulo (N)", style={'fontWeight': 'bold', 'color': '#2980b9'}),
                        dcc.Slider(id='sld-n', min=5, max=100, step=1, value=40, marks={10:'10', 50:'50', 100:'100'}, tooltip={"placement": "bottom"})
                    ]),
                    html.Div(style=slider_style_log, children=[
                        html.Label("⚡ Vel. Pattuglia (kts)", style={'fontWeight': 'bold', 'color': '#2980b9'}),
                        dcc.Slider(id='sld-v', min=0.5, max=2.0, step=0.1, value=1.5, marks={0.5:'0.5', 1.5:'1.5'}, tooltip={"placement": "bottom"})
                    ]),
                    html.Div(style=slider_style_log, children=[
                        html.Label("📡 Portata Sensore (m)", style={'fontWeight': 'bold', 'color': '#27ae60'}),
                        dcc.Slider(id='sld-r', min=1, max=100, step=1, value=45, marks={1:'1', 50:'50', 100:'100'}, tooltip={"placement": "bottom"})
                    ]),
                ]),

                # RIGA 3: LOGISTICA
                html.Div(style={**slider_row_style_log, 'backgroundColor': '#fff7e6', 'border': '1px solid #f39c12'}, children=[
                    html.Div(style=slider_style_log, children=[
                        html.Label("🔋 Autonomia (gg)", style={'fontWeight': 'bold', 'color': '#8e44ad'}),
                        dcc.Slider(id='sld-days', min=1, max=14, step=1, value=7, marks={1:'1', 7:'7'}, tooltip={"placement": "bottom"})
                    ]),
                    html.Div(style=slider_style_log, children=[
                        html.Label("⏳ Tempo Ricarica (h)", style={'fontWeight': 'bold', 'color': '#d35400'}),
                        dcc.Slider(id='sld-t-charge', min=0.5, max=4.0, step=0.1, value=1.5, marks={0.5:'Fast', 1.5:'Std', 4:'Eco'}, tooltip={"placement": "bottom"})
                    ]),
                    html.Div(style=slider_style_log, children=[
                        html.Label("💶 Costo Veicolo (k€)", style={'fontWeight': 'bold', 'color': '#7f8c8d'}),
                        dcc.Slider(id='sld-cost-uuv', min=5, max=40, step=1, value=15, marks={10:'10k', 30:'30k'}, tooltip={"placement": "bottom"})
                    ]),
                    html.Div(style=slider_style_log, children=[
                        html.Label("🔋 Costo Batteria (€)", style={'fontWeight': 'bold', 'color': '#7f8c8d'}),
                        dcc.Slider(id='sld-cost-batt', min=500, max=3000, step=100, value=2000, marks={500:'0.5k', 2000:'2k'}, tooltip={"placement": "bottom"})
                    ]),
                ]),

                # RIGA 4: INTERDIZIONE
                html.Div(style={**slider_row_style_log, 'backgroundColor': '#ffeaea'}, children=[
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

                # AREA GRAFICI LOGISTICA
                html.Div(style={'display': 'flex', 'height': '48vh', 'alignItems': 'center', 'justifyContent': 'space-between'}, children=[
                    dcc.Graph(id='graph-3d', style={'width': '27%', 'height': '100%'}),
                    dcc.Graph(id='graph-2d', style={'width': '24%', 'height': '100%'}),
                    dcc.Graph(id='graph-capex', style={'width': '27%', 'height': '100%'}),
                    dcc.Graph(id='graph-kpi', style={'width': '18%', 'height': '90%'}),
                ])
            ])
        ]), # <-- Parentesi della discordia sistemata!

        # ----------------------------------------------------------------------
        # TAB 2: FISICA V78
        # ----------------------------------------------------------------------
        dcc.Tab(label='📡 FISICA SENSORE E PROBABILITÀ (V78)', style={'fontWeight': 'bold'}, selected_style={'fontWeight': 'bold', 'borderTop': '4px solid #1abc9c'}, children=[
            html.Div(style={'padding': '20px', 'backgroundColor': '#eef2f3'}, children=[
                
                html.Div(style={**row_style_phys, 'backgroundColor': '#e8f6f3', 'border': '1px solid #1abc9c'}, children=[
                    html.Div(style={'width': '100%', 'padding':'5px'}, children=[
                        html.Label("📡 FATTORE K (Efficienza Beamforming)", style={'fontWeight':'bold', 'color':'#16a085', 'fontSize': '14px'}),
                        dcc.Slider(id='phys-k-factor', min=0.0, max=4.0, step=0.1, value=2.0, 
                            marks={0.0: {'label': '0.0'}, 1.0: {'label': '1.0'}, 2.0: {'label': '2.0'}, 3.0: {'label': '3.0'}, 4.0: {'label': '4.0 (Attivo)'}})
                    ]),
                ]),

                html.Div(style=row_style_phys, children=[
                    html.Div(style=slider_style_phys, children=[html.Label("🚁 Flotta Totale (N)"), dcc.Slider(id='phys-n', min=20, max=300, step=10, value=120)]),
                    html.Div(style=slider_style_phys, children=[html.Label(id='phys-label-portata', style={'fontWeight': 'bold', 'color': '#8e44ad'}), dcc.Slider(id='phys-r', min=10, max=100, step=5, value=50)]),
                    html.Div(style=slider_style_phys, children=[html.Label("↔️ Offset Laterale (m)"), dcc.Slider(id='phys-off', min=0, max=100, step=5, value=15)]),
                    html.Div(style=slider_style_phys, children=[html.Label("🌍 Tratta Cavo (km)"), dcc.Slider(id='phys-len', min=2, max=50, step=2, value=5)]),
                ]),

                html.Div(style={'display': 'flex', 'height': '65vh', 'justifyContent': 'space-between'}, children=[
                    html.Div(style={'width': '59%', 'height': '100%', 'backgroundColor': 'white', 'borderRadius': '8px', 'padding': '5px'}, children=[
                        dcc.Graph(id='phys-g-map', style={'height': '100%'})
                    ]),
                    html.Div(style={'width': '40%', 'height': '100%', 'display':'flex', 'flexDirection':'column', 'justifyContent':'space-between'}, children=[
                        html.Div(style={'height': '49%', 'backgroundColor': 'white', 'borderRadius': '8px'}, children=[dcc.Graph(id='phys-g-90', style={'height': '100%'})]),
                        html.Div(style={'height': '49%', 'backgroundColor': 'white', 'borderRadius': '8px'}, children=[dcc.Graph(id='phys-g-10', style={'height': '100%'})]),
                    ]),
                ]),
            ])
        ])
    ])
])

# ==============================================================================
# 4. CALLBACKS LOGISTICA (TAB 1)
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

    (curr_capex_km, n_batt_tot, n_batt_rack, power_req_sbs, data_req_sbs, 
     cost_breakdown, cycle_eff, power_tot_sys, data_tot_sys, cable_cost_km) = K_log.calculate_system_complete(
        len_total, calc_n, days_in, c_uuv_k_in, c_batt_in, t_charge_in
    )
    
    total_capex_meuro = sum(cost_breakdown.values()) / 1000000.0
    sys_cost_k_km = curr_capex_km / 1000.0
    
    box_text = [
        html.Div(style=data_box_style_log, children=[html.Span("Assorbimento massimo di una SBS"), html.B(f"{power_req_sbs:.1f} kW", style={'color': '#d35400'})]),
        html.Div(style=data_box_style_log, children=[html.Span("Assorbimento totale da rete a terra"), html.B(f"{power_tot_sys:.1f} MW", style={'color': '#c0392b'})]),
        html.Div(style=data_box_style_log, children=[html.Span("Traffico dati massimo di una SBS"), html.B(f"{data_req_sbs:.0f} Mbps", style={'color': '#2980b9'})]),
        html.Div(style=data_box_style_log, children=[html.Span("Traffico dati massimo previsto"), html.B(f"{data_tot_sys:.1f} Gbps", style={'color': '#8e44ad'})]),
        html.Div(style=data_box_style_log, children=[html.Span("Costo chilometrico cavo (€/Km)"), html.B(f"{cable_cost_km:.0f} k€", style={'color': '#27ae60'})]),
        html.Div(style=data_box_style_log, children=[html.Span("Costo chilometrico sistema (€/Km)"), html.B(f"{sys_cost_k_km:.0f} k€", style={'color': '#16a085'})]),
        html.Div(style=data_box_style_log, children=[html.Span("CAPEX Totale"), html.B(f"{total_capex_meuro:.1f} M€", style={'color': '#2c3e50', 'fontSize': '14px'})])
    ]

    t_total, t_run, t_saved = K_log.calculate_intervention_complex(calc_z, r_in, v_sprint, t_c2, r_eff, t_eval)
    is_success = t_total <= t_target
    status_icon = "✅" if is_success else "❌"
    status_color = "green" if is_success else "red"
    status_text = "SUCCESSO" if is_success else "FALLIMENTO"
    
    N_range = np.linspace(5, 100, 30)
    V_range = np.linspace(0.5, 2.0, 30)
    X_n, Y_v = np.meshgrid(N_range, V_range)
    Z_plot = np.zeros_like(X_n)
    C_surf = np.zeros_like(X_n) 
    
    c_min_scenario, _, _, _, _, _, _, _, _, _ = K_log.calculate_system_complete(len_total, 5, 1, 5, 500, 4.0)
    c_max_scenario, _, _, _, _, _, _, _, _, _ = K_log.calculate_system_complete(len_total, 100, 14, 40, 3000, 0.5)

    for i in range(X_n.shape[0]):
        for j in range(X_n.shape[1]):
            val_z = K_log.solve_blind_time(X_n[i,j], Y_v[i,j], r_in)
            Z_plot[i,j] = 20.0 - min(val_z, 20.0) 
            c_km_pt, _, _, _, _, _, _, _, _, _ = K_log.calculate_system_complete(
                len_total, X_n[i,j], days_in, c_uuv_k_in, c_batt_in, t_charge_in
            )
            C_surf[i,j] = c_km_pt
            
    fig3d = go.Figure()
    fig3d.add_trace(go.Surface(
        z=Z_plot, x=X_n, y=Y_v, surfacecolor=C_surf, colorscale='RdYlGn_r', cmin=c_min_scenario, cmax=c_max_scenario, 
        opacity=0.9, contours={"x": {"show": True, "color":"lightgrey"}, "y": {"show": True, "color":"lightgrey"}, "z": {"show": False}}, 
        hovertemplate="Vel: %{y:.1f} kts<br>Flotta: %{x:.0f}<br>Z: %{z:.2f} min<br>CAPEX: %{surfacecolor:.0f} €/km<extra></extra>", showscale=False
    ))

    theta = np.linspace(0, 2*np.pi, 60)
    el_n = 40 + 20 * np.cos(theta)
    el_v = 1.15 + 0.35 * np.sin(theta)
    el_z_real = np.array([K_log.solve_blind_time(n, v, r_in) for n, v in zip(el_n, el_v)])
    el_z_plot = 20.0 - np.minimum(el_z_real, 20.0)
    fig3d.add_trace(go.Scatter3d(x=el_n, y=el_v, z=el_z_plot, mode='lines', line=dict(color='blue', width=6), name='Zona 20-60'))
    
    pt_z_plot = 20.0 - min(calc_z, 20.0)
    fig3d.add_trace(go.Scatter3d(x=[calc_n], y=[v_in], z=[pt_z_plot], mode='markers', marker=dict(size=6, color='red', line=dict(color='black', width=1)), name='Attuale', hoverinfo='skip'))

    annotations = [dict(
        x=calc_n, y=v_in, z=pt_z_plot,
        text=(f"<b>Combinazione Attuale</b><br>Velocità: {v_in} kts<br>Flotta: {int(calc_n)}<br>Tempo Cieco: {calc_z:.2f} min<br>Costo/km: {sys_cost_k_km:.0f} k€<br><span style='color:{status_color}'><b>{status_icon} {status_text}</b></span>"),
        showarrow=True, arrowhead=1, arrowsize=1, arrowwidth=2, ax=-100, ay=-80, bgcolor="white", bordercolor="red", borderwidth=3, opacity=0.9, font=dict(color="black", size=11), align="left"
    )]

    scene_config = dict(
        xaxis=dict(title='Flotta (N)', showbackground=True, backgroundcolor='#f4f6f7', gridcolor='#bdc3c7', range=[5, 100], autorange=False),
        yaxis=dict(title='Velocità (kts)', showbackground=True, backgroundcolor='#f4f6f7', gridcolor='#bdc3c7', range=[0.5, 2.0], autorange=False),
        zaxis=dict(title='Tempo Cieco (min)', showbackground=True, backgroundcolor='#f4f6f7', gridcolor='#bdc3c7', range=[0, 20], autorange=False, tickmode='array', tickvals=[0,10,20], ticktext=["20","10","0"]),
        annotations=annotations
    )
    if trigger is None: scene_config['camera'] = dict(projection=dict(type="orthographic"), eye=dict(x=-1.5, y=-1.5, z=1.2))

    fig3d.update_layout(title="<b>TRADE-OFF</b>", uirevision='TheRock', scene=scene_config, margin=dict(l=5, r=5, t=30, b=5), showlegend=True, legend=dict(x=0, y=1, orientation="h", bgcolor='rgba(255,255,255,0.5)'))

    range_r = np.linspace(1, 100, 100)
    z_curve = [K_log.solve_blind_time(calc_n, v_in, r) for r in range_r]
    fig2d = go.Figure()
    fig2d.add_trace(go.Scatter(x=range_r, y=z_curve, mode='lines', line=dict(color='#2980b9', width=3), name='Curva portata/tempo cieco'))
    fig2d.add_trace(go.Scatter(x=[r_in], y=[calc_z], mode='markers+text', marker=dict(size=10, color='red'), text=[f"{r_in}m"], textposition="top center", name='Portata Attuale'))
    fig2d.update_layout(title="<b>PORTATA</b>", xaxis_title='Range (m)', yaxis_title='Z (min)', plot_bgcolor="white", margin=dict(l=30, r=10, t=30, b=30), showlegend=True, legend=dict(x=0.5, y=1.1, orientation="h", xanchor="center"))
    fig2d.update_yaxes(autorange="reversed")

    labels = list(cost_breakdown.keys())
    values = [v / 1000000.0 for v in cost_breakdown.values()] 
    fig_capex = go.Figure(go.Bar(x=labels, y=values, marker_color=['#3498db', '#9b59b6', '#34495e', '#f1c40f', '#e74c3c'], text=[f"{v:.1f} M€" for v in values], textposition='auto'))
    fig_capex.update_layout(title="<b>CAPEX (M€)</b>", yaxis=dict(title='M€', showgrid=True, gridcolor='lightgrey'), xaxis=dict(showgrid=False), plot_bgcolor='white', margin=dict(l=30, r=10, t=30, b=30))

    fig_kpi = go.Figure(go.Bar(x=['Tot', 'Buffer'], y=[n_batt_tot, n_batt_rack], marker_color=['#2ecc71', '#f39c12'], text=[n_batt_tot, n_batt_rack], textposition='auto'))
    fig_kpi.update_layout(title="<b>BATTERIE (Mod)</b>", yaxis=dict(showgrid=False), xaxis=dict(showgrid=False), plot_bgcolor='white', margin=dict(l=20, r=20, t=30, b=20))

    return out_z, out_n, fig3d, fig2d, fig_capex, fig_kpi, box_text

# ==============================================================================
# 5. CALLBACK FISICA V78 (TAB 2)
# ==============================================================================
@app.callback(
    [Output('phys-g-map', 'figure'), Output('phys-g-90', 'figure'), Output('phys-g-10', 'figure'), Output('phys-label-portata', 'children')],
    [Input('phys-n', 'value'), Input('phys-r', 'value'), Input('phys-off', 'value'), Input('phys-len', 'value'), Input('phys-k-factor', 'value')]
)
def update_physics(N, R_teo, Offset, L_km, K_val):
    try:
        R_eff = K_phys.get_R_eff(R_teo, float(K_val))
        label_text = f"🎯 Portata Utile >10%: {R_eff:.1f} m (Teorica: {R_teo}m)"

        Y_max = R_teo + Offset + 10
        Y = np.linspace(-Y_max, Y_max, 200)
        X = [0, 100]
        
        P = K_phys.calculate_probability_profile(np.abs(Y), N, L_km, R_teo, Offset, float(K_val))
        Z = np.column_stack((P, P))
        
        colors = [[0.00, '#000000'], [0.005, '#0000FF'], [0.10, '#00FFFF'], [0.40, '#00FF00'], [0.80, '#FFFF00'], [1.00, '#FF0000']]
        fig_map = go.Figure(data=go.Heatmap(z=Z, x=X, y=Y, colorscale=colors, zmin=0, zmax=100))
        fig_map.add_shape(type="line", x0=0, x1=100, y0=Offset, y1=Offset, line=dict(color="white", width=1, dash="dash"))
        fig_map.add_shape(type="line", x0=0, x1=100, y0=-Offset, y1=-Offset, line=dict(color="white", width=1, dash="dash"))
        fig_map.add_shape(type="line", x0=0, x1=100, y0=0, y1=0, line=dict(color="lime", width=3))
        fig_map.add_shape(type="line", x0=0, x1=100, y0=Offset+R_eff, y1=Offset+R_eff, line=dict(color="gray", width=1, dash="dot"))
        fig_map.add_shape(type="line", x0=0, x1=100, y0=-(Offset+R_eff), y1=-(Offset+R_eff), line=dict(color="gray", width=1, dash="dot"))
        
        fig_map.update_layout(title=f"CAMPO PROBABILITÀ (R utile = {R_eff:.1f}m)", margin=dict(l=40, r=20, t=40, b=20), plot_bgcolor='#111', xaxis=dict(showticklabels=False), yaxis=dict(title="Distanza Laterale (m)"))

        N_rng = np.linspace(20, 300, 20)
        Off_rng = np.linspace(0, 100, 20)
        X_off, Y_n = np.meshgrid(Off_rng, N_rng)
        Z_90, Z_10 = np.zeros_like(X_off), np.zeros_like(X_off)
        
        for i in range(len(N_rng)):
            for j in range(len(Off_rng)):
                d90 = K_phys.find_iso_distance(N_rng[i], L_km, R_teo, Off_rng[j], 90.0, float(K_val))
                d10 = K_phys.find_iso_distance(N_rng[i], L_km, R_teo, Off_rng[j], 10.0, float(K_val))
                Z_90[i,j] = max(0, d90)
                Z_10[i,j] = d10

        curr90 = K_phys.find_iso_distance(N, L_km, R_teo, Offset, 90.0, float(K_val))
        fig90 = go.Figure(go.Surface(z=Z_90, x=X_off, y=Y_n, colorscale='Reds', showscale=False))
        mk_col = 'yellow' if curr90 >= 0 else 'black'
        fig90.add_trace(go.Scatter3d(x=[Offset], y=[N], z=[max(0, curr90)], mode='markers', marker=dict(size=6, color=mk_col, line=dict(width=2, color='white'))))
        fig90.update_layout(title="ZONA SICURA (>90%)", scene=dict(camera=CAM_3D, xaxis_title="Offset", yaxis_title="N", zaxis_title="m"), margin=dict(l=0, r=0, t=30, b=0))

        curr10 = K_phys.find_iso_distance(N, L_km, R_teo, Offset, 10.0, float(K_val))
        fig10 = go.Figure(go.Surface(z=Z_10, x=X_off, y=Y_n, colorscale='Blues', showscale=False))
        fig10.add_trace(go.Scatter3d(x=[Offset], y=[N], z=[curr10], mode='markers', marker=dict(size=6, color='cyan', line=dict(width=2, color='white'))))
        fig10.update_layout(title="ZONA WARNING (>10%)", scene=dict(camera=CAM_3D, xaxis_title="Offset", yaxis_title="N", zaxis_title="m"), margin=dict(l=0, r=0, t=30, b=0))

        return fig_map, fig90, fig10, label_text

    except Exception as e:
        print("ERRORE TAB 2:", e)
        traceback.print_exc()
        return go.Figure(), go.Figure(), go.Figure(), "Errore Calcolo"

if __name__ == '__main__':
    app.run(debug=True, use_reloader=False)