// 定数とデータ定義のファイル
// SVG設定を定数として定義
export const SVG_CONFIG = {
    width: 800,
    height: 600,
    groundLevel: 550,
    centerX: 400,
  };
  
  // モータースラストデータ
  export const MOTOR_THRUST_DATA = {
    '1/2A6-2': [0.000,0.445,0.891,1.336,1.782,2.227,2.673,3.118,3.564,4.009,4.455,3.345,2.236,2.012,1.789,1.565,1.341,1.118,1.118,1.118,1.118,1.118,1.118,1.118,1.118,1.118,1.118,1.118,1.118,1.118,0.894,0.671,0.447,0.224],
    'A8-3': [0.000,0.891,1.782,2.673,3.564,4.455,5.345,6.236,7.127,8.018,8.909,9.800,6.900,4.000,3.600,3.200,2.800,2.400,2.400,2.400,2.400,2.400,2.400,2.400,2.400,2.400,2.400,2.400,2.400,2.400,2.400,2.400,2.400,1.600,0.800],
    'B6-4': [0.000,1.782,3.564,5.345,7.127,8.909,10.691,12.473,14.255,16.036,17.818,19.600,13.800,8.000,7.200,6.400,5.600,4.800,4.800,4.800,4.800,4.800,4.800,4.800,4.800,4.800,4.800,4.800,4.800,4.800,4.800,3.200,1.600]
  };
  
  // パラシュートサイズデータ (直径mm)
  export const PARACHUTE_SIZES = {
    'φ180': 180,
    'φ250': 250,
    'φ300': 300,
    'φ600': 600,
    'φ900': 900
  };
  
  // フィン材料のデータ
  export const FIN_MATERIALS = {
    'light_balsa': { name: '軽量バルサ', E: 2450000000, G: 85750000 },
    'balsa': { name: 'バルサ', E: 3000000000, G: 120000000 },
    'light_veneer': { name: '軽量ベニア', E: 8000000000, G: 450000000 }
  };
  
  // ノーズ形状と抗力係数
  export const NOSE_SHAPES = {
    'cone': { name: '円錐', cd: 0.83 },
    'parabola': { name: '放物線', cd: 0.7 },
    'ogive': { name: 'オジブ', cd: 0.61 }
  };
  
  // 風速プロファイルの種類定義 - 表記を簡略化
  export const WIND_PROFILES = {
    'uniform': { name: '一定（変化なし）', alpha: 0 },
    'openSea': { name: '海上・平原', alpha: 0.12 },
    'farmland': { name: '農地・草原', alpha: 0.2 },
    'suburban': { name: '郊外・小さな町', alpha: 0.3 },
    'urban': { name: '都市部・森林', alpha: 0.4 }
  };
  
  // 物理定数
  export const PHYSICAL_CONSTANTS = {
    launchRailLength: 0.65, // 発射台の長さ (m)
  };
  
  // 角度の応答時間とステップ数
  export const ANGLE_RESPONSE_DT = 0.2; // 角度応答時間 (s)
  export const ANGLE_STEPS_PER_UPDATE = 10; // dt = 0.02に対して10ステップ (0.2秒)
  
  // アニメーション速度係数（1.0が実時間）
  export const ANIMATION_SPEED = 0.2; // 0.2倍速でより詳細に観察
  
  // スライダーの応答性
  export const SLIDER_DEBOUNCE_TIME = 0; // 0ミリ秒に変更して即時反応させる
  
  // 単位変換関数
  export const mmToM = (mm) => mm / 1000;
  export const gToKg = (g) => g / 1000;
  
  // 解析ビューの設定
  export const ANALYSIS_VIEW_CONFIG = {
    width: 400,
    height: 500,
    padding: 20,
    maxScale: 2.0,
    minScale: 0.8
  };
  
  // UI設定
  export const UI_CONFIG = {
    defaultTab: 'design',
    designViewWidth: 300,
    designViewHeight: 400,
    analysisViewWidth: 200,
    analysisViewHeight: 400
  };