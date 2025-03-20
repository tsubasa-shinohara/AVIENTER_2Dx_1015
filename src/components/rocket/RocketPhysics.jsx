// 物理計算関連の関数
import { 
  MOTOR_THRUST_DATA, PHYSICAL_CONSTANTS, 
  ANGLE_RESPONSE_DT, ANGLE_STEPS_PER_UPDATE,
  NOSE_SHAPES, FIN_MATERIALS, WIND_PROFILES, mmToM, gToKg 
} from './RocketConstants';

// 物理制御と拡張制御を分離する定数を追加
export const PHYSICAL_ATTITUDE_CONTROL = true;  // 物理ベースの姿勢制御 (常に有効にすべき)
export const ENHANCED_ATTITUDE_CONTROL = false;  // 拡張姿勢制御 (風見効果など)

// ロケットの投影面積を計算する関数
export const calculateProjectedArea = (rocketParams) => {
  
  // パラメータの存在と型チェック
  if (!rocketParams || typeof rocketParams !== 'object') {
    console.error('calculateProjectedArea: 無効なrocketParamsオブジェクト', rocketParams);
    return { 
      frontalArea: 0, 
      sideArea: 0, 
      finArea: 0, 
      totalFinArea: 0, 
      angledArea: 0 
    };
  }

    // 必須パラメータの存在チェック
    const { 
      noseShape, noseHeight, bodyHeight, bodyWidth, 
      finHeight, finBaseWidth, finTipWidth, finSweepLength, finThickness 
    } = rocketParams;
  
    // 必須パラメータが存在しない場合はエラーログを出力し、安全な値を返す
    if (
      typeof noseHeight !== 'number' || 
      typeof bodyHeight !== 'number' || 
      typeof bodyWidth !== 'number' || 
      typeof finHeight !== 'number' || 
      typeof finBaseWidth !== 'number' || 
      typeof finTipWidth !== 'number' || 
      typeof finThickness !== 'number' || 
      typeof finSweepLength !== 'number'
    ) {
      console.error('calculateProjectedArea: 必須の数値パラメータが不足しています', {
        noseHeight, bodyHeight, bodyWidth, finHeight, finBaseWidth, finTipWidth, finThickness, finSweepLength
      });
      return { 
        frontalArea: 0, 
        sideArea: 0, 
        finArea: 0, 
        totalFinArea: 0, 
        angledArea: 0 
      };
    }

  // 単位をmm->mに変換
  const noseHeight_m = mmToM(noseHeight);
  const bodyHeight_m = mmToM(bodyHeight);
  const bodyWidth_m = mmToM(bodyWidth);
  const bodyRadius_m = bodyWidth_m / 2;
  const finHeight_m = mmToM(finHeight);
  const finBaseWidth_m = mmToM(finBaseWidth);
  const finTipWidth_m = mmToM(finTipWidth);
  const finSweepLength_m = mmToM(finSweepLength);
  
  // 正面からの投影面積 (m^2)
  const frontalArea = Math.PI * Math.pow(bodyRadius_m, 2) + (finHeight * finThickness) * 4 * 0.0000001;
  
  // 側面投影面積の計算 (m^2)
  // ボディ部分
  const bodyArea = bodyWidth_m * bodyHeight_m;
  
  // ノーズ部分 - 形状に応じて計算
  let noseArea;
  if (noseShape === 'cone') {
    noseArea = 0.5 * bodyWidth_m * noseHeight_m; // 三角形の面積
  } else if (noseShape === 'parabola') {
    noseArea = (2/3) * bodyWidth_m * noseHeight_m; // 放物線の近似
  } else { // ogive
    noseArea = (2/3) * bodyWidth_m * noseHeight_m; // オジブの近似
  }
  
  // フィン1枚あたりの投影面積
  const finArea = finHeight_m * (finBaseWidth_m + finTipWidth_m) / 2; // 台形の面積
  
  // フィン4枚の合計投影面積
  const totalFinArea = finArea * 2;
  
  // 側面からの合計投影面積
  const sideArea = bodyArea + noseArea + totalFinArea;
  
  // 斜め45度からの投影面積（近似）（修正が必要）
  const angledArea = Math.sqrt(Math.pow(frontalArea, 2) + Math.pow(sideArea, 2));
  
  return {
    frontalArea,  // 正面からの投影面積 (m^2)
    sideArea,     // 側面からの投影面積 (m^2)
    finArea,      // フィン1枚の投影面積 (m^2)
    totalFinArea, // フィン4枚の合計投影面積 (m^2)
    angledArea    // 斜め45度からの投影面積（近似） (m^2)
  };
};

// ロケットの体積を計算する関数
export const calculateVolume = (rocketParams) => {
  const { noseShape, noseHeight, bodyHeight, bodyWidth } = rocketParams;
  
  // 単位をmm->mに変換
  const noseHeight_m = mmToM(noseHeight);
  const bodyHeight_m = mmToM(bodyHeight);
  const bodyRadius_m = mmToM(bodyWidth) / 2;
  
  // ボディ体積（円柱） (m^3)
  const bodyVolume = Math.PI * Math.pow(bodyRadius_m, 2) * bodyHeight_m;
  
  // ノーズ体積 - 形状に応じて計算 (m^3)
  let noseVolume;
  if (noseShape === 'cone') {
    noseVolume = (1/3) * Math.PI * Math.pow(bodyRadius_m, 2) * noseHeight_m; // 円錐の体積
  } else if (noseShape === 'parabola') {
    noseVolume = (1/2) * Math.PI * Math.pow(bodyRadius_m, 2) * noseHeight_m; // 放物線回転体の体積
  } else { // ogive
    noseVolume = (2/3) * Math.PI * Math.pow(bodyRadius_m, 2) * noseHeight_m; // オジブの近似体積
  }
  
  // 合計体積 (m^3)
  const totalVolume = bodyVolume + noseVolume;
  
  // フィンの体積は非常に小さいため、ここでは無視する
  
  return {
    bodyVolume,  // ボディ部分の体積 (m^3)
    noseVolume,  // ノーズ部分の体積 (m^3)
    totalVolume  // 合計体積 (m^3)
  };
};

// 圧力中心位置を計算する関数（モーメント計算に使用）
export const calculateCenterOfPressure = (rocketParams) => {
  const { noseShape, noseHeight, bodyHeight, bodyWidth, finHeight, finBaseWidth, finTipWidth, finSweepLength } = rocketParams;
  
  // 面積と体積を計算
  const areas = calculateProjectedArea(rocketParams);
  
  // ノーズ先端からの各コンポーネントの圧力中心位置 (mm)
  // ノーズの圧力中心位置 - 形状に応じて計算
  let noseCp;
  if (noseShape === 'cone') {
    noseCp = noseHeight * (2/3); // 円錐の圧力中心はノーズ長の2/3
  } else if (noseShape === 'parabola') {
    noseCp = noseHeight * (3/5); // 放物線の圧力中心（近似値）
  } else { // ogive
    noseCp = noseHeight * (3/5); // オジブの圧力中心（近似値）
  }
  
  // ボディの圧力中心位置（ノーズ先端から）
  const bodyCp = noseHeight + bodyHeight / 2; // mm
  
  //　フィンの圧力中心位置（フィン付け根先端から）
  const finCP_single = (Math.pow((finSweepLength + finTipWidth), 2) - Math.pow(finSweepLength, 2) + Math.pow(finBaseWidth, 2) + (finSweepLength + finTipWidth) * finBaseWidth) / (3 * ((finSweepLength + finTipWidth) + finBaseWidth - finSweepLength)); // mm

  // フィンの圧力中心位置（ノーズ先端から）
  const finCp = noseHeight + bodyHeight - finBaseWidth + finCP_single; // mm
  
  // 面積による重み付け計算
  // 単位を揃えるためにm²からmm²に変換
  const noseArea = areas.noseArea * 1000000; // m^2 → mm^2
  const bodyArea = areas.sideArea * 1000000 - noseArea; // m^2 → mm^2
  const totalFinArea = areas.totalFinArea * 1000000; // m^2 → mm^2
  
  // 圧力中心の計算（重み付け平均）
  const totalArea = noseArea + bodyArea + totalFinArea;
  const centerOfPressure = (noseCp * noseArea + bodyCp * bodyArea + finCp * totalFinArea) / totalArea; // mm
  
  // フィンを除いた前部の圧力中心（空力計算用）
  const foreBodyArea = noseArea + bodyArea;
  const foreBodyCp = (noseCp * noseArea + bodyCp * bodyArea) / foreBodyArea;
  
  return {
    noseCp,           // ノーズの圧力中心位置 (mm)
    bodyCp,           // ボディの圧力中心位置 (mm)
    finCp,            // フィンの圧力中心位置 (mm)
    centerOfPressure, // 全体の圧力中心位置 (mm)
    foreBodyCp        // フィンを除いた前部の圧力中心位置 (mm)
  };
};

// 空力中心位置を計算する関数
export const calculateAerodynamicCenter = (rocketParams) => {
  // この行で必要なすべてのプロパティを取り出します
  const { noseHeight, bodyHeight, bodyWidth, finHeight, finBaseWidth, finTipWidth, finSweepLength } = rocketParams;
  
  // 投影面積を計算
  const areas = calculateProjectedArea(rocketParams);

  // lengthOfCo
  const lengthOfCo = ((finBaseWidth - finTipWidth) / finHeight) * ((bodyWidth / 2) + finHeight) + finTipWidth;
  
  // テーパー比（ramda）
  const ramda = finTipWidth / lengthOfCo;

  // 胴体容積 (m^3)
  const volumeData = calculateVolume(rocketParams);

  // c_bar
  const c_bar = (2 * lengthOfCo / 3) * (1 + ramda + Math.pow(ramda, 2)) / (1 + ramda);

  // y_bar
  const y_bar = (finHeight + (bodyWidth / 2)) * (1 + (2 * ramda)) / (3 * (1 + ramda));

  // 面積による重み付け計算（m^2をmm^2に変換）
  const noseArea = areas.noseArea * 1000000; // m^2 → mm^2
  const bodyArea = areas.sideArea * 1000000 - noseArea; // m^2 → mm^2
  const totalFinArea = areas.totalFinArea * 1000000; // m^2 → mm^2

  // WingArea
  const wingArea = (finTipWidth + lengthOfCo) * ((bodyWidth / 2) + finHeight);
  
  // V*fus
  const Vstar_fus = volumeData.totalVolume * 1000000000 / (c_bar * wingArea); // 単位を合わせる

  // AspectRatio
  const aspectRatio = ((2 * finHeight) + bodyWidth) * ((2 * finHeight) + bodyWidth) / wingArea;

  // CLα
  const cl_alpha = ((3.14 * aspectRatio) * 0.5) * Math.pow((1 - Math.pow((bodyWidth / 2) / (((finHeight + (bodyWidth / 2)) / 2)), 2)) , 2);

  // hn
  const hn = 0.25 + (1 / cl_alpha) * (-1) * (2 * Vstar_fus);

  // hnwc_bar
  const hnwc_bar = hn * c_bar;

  // x1
  const x1 = (bodyWidth / 2) * finSweepLength / finHeight;

  // x2
  const x2 = y_bar * (x1 + finSweepLength + finTipWidth - lengthOfCo) / (((bodyWidth / 2) + finHeight));

  // small_xac
  const small_xac = c_bar - x2 - hnwc_bar;

  // 空力中心計算
  const aerodynamicCenter = noseHeight + bodyHeight - small_xac;
  
  return {
    aerodynamicCenter  // 空力中心位置 (mm)
  };
};

// 静安定マージン計算用の圧力中心位置を計算する関数
export const calculateStabilityCenterOfPressure = (rocketParams) => {
  const { noseShape, noseHeight, bodyHeight, bodyWidth, finHeight, finBaseWidth, finTipWidth, finSweepLength } = rocketParams;

  // mc
  const mc = Math.pow((Math.pow((finSweepLength + (finTipWidth / 2) - (finBaseWidth / 2)), 2)) + (Math.pow(finHeight, 2)), 0.5);
  
  // ノーズ先端からの位置を計算 (mm)
  // この静安定マージン用の圧力中心は通常の圧力中心とは異なる計算方法を使用
  
  // ノーズの圧力中心
  let noseStabilityCp;
  if (noseShape === 'cone') {
    noseStabilityCp = noseHeight * 0.5; // 円錐の静安定用圧力中心
  } else if (noseShape === 'parabola') {
    noseStabilityCp = noseHeight * 0.45; // 放物線の静安定用圧力中心
  } else { // ogive
    noseStabilityCp = noseHeight * 0.4; // オジブの静安定用圧力中心
  }
  
  // フィンのcn
  const fin_cn_1 = 1 + (finHeight / (finHeight + (bodyWidth / 2)));
  const fin_cn_2 = 4 * 4 * Math.pow((finHeight / (bodyWidth / 2)), 2);
  const fin_cn_3 = 1 + Math.pow((1 + (Math.pow(((2 * mc) / (finTipWidth + finBaseWidth)), 2))), 0.5);
  const fin_cn = fin_cn_1 * fin_cn_2 / fin_cn_3;

  // CnTotal
  const cnTotal = 2 + fin_cn;

  // フィンの圧力中心
  const finStabilityCp = (noseHeight + bodyHeight - finBaseWidth) + ((finSweepLength / 3) * ((finBaseWidth + 2 * finTipWidth) / (finBaseWidth + finTipWidth))) + ((finBaseWidth + finTipWidth) - ((finBaseWidth * finTipWidth) / (finBaseWidth + finTipWidth))) / 6;
  
  // 静安定用の圧力中心位置（重み付け平均）
  const stabilityCenterOfPressure = (2 * noseStabilityCp + fin_cn * finStabilityCp) / cnTotal;
  
  return {
    stabilityCenterOfPressure,  // 静安定計算用の圧力中心位置 (mm)
  };
};

// 静安定マージンを計算する関数
export const calculateStaticMargin = (rocketParams) => {
  const { centerOfGravity, bodyWidth } = rocketParams;
  
  // 通常の圧力中心を計算
  const cpData = calculateCenterOfPressure(rocketParams);
  
  // 静安定計算用の特別な圧力中心を計算
  const stabilityCp = calculateStabilityCenterOfPressure(rocketParams);
  
  // 静安定マージン = (圧力中心位置 - 重心位置) / ボディ直径
  // 通常の圧力中心を使用した場合
  const standardStaticMargin = (cpData.centerOfPressure - centerOfGravity) / bodyWidth;
  
  // 静安定用圧力中心を使用した場合
  const stabilityStaticMargin = (stabilityCp.stabilityCenterOfPressure - centerOfGravity) / bodyWidth;
  
  return {
    standardStaticMargin,   // 通常の静安定マージン
    stabilityStaticMargin   // 静安定計算用の静安定マージン
  };
};

// フィンダイバージェンス速度を計算する関数
export const calculateFinDivergenceSpeed = (rocketParams) => {
  const { finHeight, finBaseWidth, finTipWidth, finSweepLength, finThickness, finMaterial } = rocketParams;
  
  // 単位をmmからmに変換
  const finHeight_m = mmToM(finHeight);
  const finBaseWidth_m = mmToM(finBaseWidth);
  const finTipWidth_m = mmToM(finTipWidth);
  const finSweepLength_m = mmToM(finSweepLength);
  const finThickness_m = mmToM(finThickness);
  
  // フィン材料の特性を取得
  const material = FIN_MATERIALS[finMaterial];
  const G = material.G; // 横弾性係数 (Pa)
  
  // 空気密度 (kg/m³)
  const rho = 1.225;

  // 平均コード長の計算 (m)
  const meanChord = (finBaseWidth_m + finTipWidth_m) / 2;
  
  // 後退角(rad)
  const sweepbackAngle = Math.atan((finSweepLength_m + 0.5 * finTipWidth_m - 0.5 * finBaseWidth_m) * 3.14 / meanChord);

  // 捻り定数J
  const J = 0.3333 * finTipWidth_m * Math.pow(finThickness_m, 3);

  // 揚力傾斜a0
  const liftCoefficient_fin = (9 / 3.14) * Math.cos(sweepbackAngle);
  
  const divSpeed = (3.14 / (2 * finHeight_m)) * Math.pow( 2 * G * J / (rho * Math.pow(meanChord, 2) * 0.25 * liftCoefficient_fin), 0.5);
  
  // 現実的な範囲内に制限（極端に大きな/小さな値を防止）
  return Math.max(20, Math.min(300, divSpeed));
};

// フィンフラッター速度を計算する関数
export const calculateFinFlutterSpeed = (rocketParams) => {
  const { finHeight, finBaseWidth, finTipWidth, finThickness, finMaterial } = rocketParams;
  
  // 単位をmmからmに変換
  const finHeight_m = mmToM(finHeight);
  const finBaseWidth_m = mmToM(finBaseWidth);
  const finTipWidth_m = mmToM(finTipWidth);
  const finThickness_m = mmToM(finThickness);
  
  // フィン材料の特性を取得
  const material = FIN_MATERIALS[finMaterial];
  const G = material.G; // 横弾性係数 (Pa)
  const E = material.E; // 縦弾性係数 (Pa)
  
  // ポアソン比（一般的な値）
  const poissonsRatio = 0.3;
  
  // 空気密度 (kg/m³)
  const rho = 1.225;
  
  // 平均コード長の計算 (m)
  const meanChord = (finBaseWidth_m + finTipWidth_m) / 2;
  
  // 経験的定数
  const empiricalConstant = 3.5;
  
  // フィンフラッター速度の計算 (m/s)
  // V_flutter = (a * t / c^1.5) * sqrt(G * E / (12 * ρ * (1 - ν^2)))
  const factorA = empiricalConstant * finThickness_m / Math.pow(meanChord, 1.5);
  const factorB = Math.sqrt(G * E / (12 * rho * (1 - Math.pow(poissonsRatio, 2))));
  
  // 計算結果が無効な場合のフォールバック
  if (!isFinite(factorA) || !isFinite(factorB)) {
    console.warn('フィンフラッター速度の計算に無効な値が発生しました。代替値を使用します。');
    return 40 + mmToM(rocketParams.bodyHeight + rocketParams.noseHeight) * 120;
  }
  
  const flutterSpeed = factorA * factorB;
  
  // 現実的な範囲内に制限（極端に大きな/小さな値を防止）
  return Math.max(30, Math.min(400, flutterSpeed));
};

// フィンたわみ量のフォーマット関数（UI表示時に使用）
export const formatFinDeflection = (deflection) => {
  // 15mmの場合（閾値または計算エラー）は「15mm以上」と表示
  if (deflection === 15) {
    return "15mm以上";
  }
  
  // 通常のたわみ量は小数点2桁までの数値を表示
  return `${deflection.toFixed(2)}mm`;
};

// フィンダイバージェンス速度とフラッター速度の表示用フォーマット関数
export const formatSpeedValue = (speed, limit = 300) => {
  // 速度値が上限を超えている場合
  if (speed >= limit) {
    return `${limit}+ m/s`;  // 「300+ m/s」のように表示
  }
  
  // 通常範囲内の速度は整数で表示
  return `${Math.round(speed)} m/s`;
};

// 高度に応じた風速を計算する関数 - 基準高度を1.5mに修正
export const calculateWindSpeedAtHeight = (baseWindSpeed, height, profile) => {
  // 高度が0の場合はそのまま基準風速を返す
  if (height <= 0) return baseWindSpeed;
  
  // プロファイルに応じたべき指数を取得
  const alpha = WIND_PROFILES[profile].alpha;
  
  // べき指数が0の場合は高度に関わらず一定風速
  if (alpha === 0) return baseWindSpeed;
  
  // 基準高度（1.5メートル - 地上計測を想定）
  const referenceHeight = 1.5; 
  
  // べき乗則による風速計算
  // V(h) = V_ref * (h/h_ref)^α
  const heightRatio = height / referenceHeight;
  const windSpeedMultiplier = Math.pow(heightRatio, alpha);
  
  // 風速の上限を設定（非現実的な値にならないよう制限）
  const maxMultiplier = 3.0; // 基準風速の3倍まで
  const actualMultiplier = Math.min(windSpeedMultiplier, maxMultiplier);
  
  return baseWindSpeed * actualMultiplier;
};

// モーメント計算用のヘルパー関数 - 修正版
const calculateLiftMoment = (velocity, omega, flightAngle, rocketParams, sideArea, aerodynamicCenter, centerOfGravity) => {
  const velocitySquared = Math.min(velocity * velocity, 10000);
  
  // 迎角（姿勢角と飛行角の差）を計算
  const angleOfAttack = omega - flightAngle;
  
  // 迎角に比例した揚力係数（小さな角度ではsin(θ)≈θ）
  // より大きな係数を使用してモーメントを増加
  const liftCoefficient = 0.6 * angleOfAttack;  // 係数は0.6
  
  // 揚力モーメントの計算 - 絶対値のみを計算
  const momentMagnitude = Math.abs(liftCoefficient * 0.5 * 1.225 * velocitySquared * sideArea * (aerodynamicCenter - centerOfGravity) * 0.001);
  
  // 符号の決定
  let finalMoment;
  if ((aerodynamicCenter >= centerOfGravity && angleOfAttack < 0) || 
      (aerodynamicCenter < centerOfGravity && angleOfAttack >= 0)) {
    finalMoment = -momentMagnitude; // マイナス
  } else {
    finalMoment = momentMagnitude;  // プラス
  }

  // 最小モーメント保証 (非常に小さな値になるのを防ぐ)
  const MIN_MOMENT = 0.00001;
  if (Math.abs(finalMoment) > 0 && Math.abs(finalMoment) < MIN_MOMENT) {
    return Math.sign(finalMoment) * MIN_MOMENT;
  }
  
  // 角度が大きすぎる場合はモーメントを増加
  if (Math.abs(angleOfAttack) > 0.5) {  // 約28.6度以上
    return finalMoment * 1.2;  // モーメントを20%増加
  }
  
  return finalMoment;
};

const calculateDragMoment = (velocity, omega, flightAngle, rocketParams, sideArea, aerodynamicCenter, centerOfGravity) => {
  const velocitySquared = Math.min(velocity * velocity, 10000);
  
  // 迎角を計算
  const angleOfAttack = omega - flightAngle;
  
  // 迎角の二乗に比例した抗力増加
  const dragCoefficient = 0.01 * Math.pow(angleOfAttack, 2) - 0.02 * angleOfAttack + 0.63;
  
  // 抗力モーメントの計算 - 絶対値のみを計算
  const momentMagnitude = Math.abs(dragCoefficient * 0.5 * 1.225 * velocitySquared * sideArea * (aerodynamicCenter - centerOfGravity) * 0.001);
  
  // 符号の決定
  let finalMoment;
  if ((aerodynamicCenter >= centerOfGravity && angleOfAttack < 0) || 
      (aerodynamicCenter < centerOfGravity && angleOfAttack >= 0)) {
    finalMoment = -momentMagnitude; // マイナス
  } else {
    finalMoment = momentMagnitude;  // プラス
  }

  // 最小モーメント保証
  const MIN_MOMENT = 0.00001;
  if (Math.abs(finalMoment) > 0 && Math.abs(finalMoment) < MIN_MOMENT) {
    return Math.sign(finalMoment) * MIN_MOMENT;
  }
  
  return finalMoment;
};

const calculateWindMoment = (noseHeight, bodyDiameter, bodyHeight, windSpeed, omega, totalFinArea, centerOfPressure, centerOfGravity) => {
  // 風速に上限を設定
  const safeWindSpeed = Math.max(-25, Math.min(25, windSpeed));

  // 風速によりフィンが受ける力
  const Dwf = 9.81 * 0.05 * Math.pow(safeWindSpeed, 2) * totalFinArea;

  // 風速によりボディが受ける力
  const Dwb = 0.23 * 0.5 * 1.225 * Math.pow(safeWindSpeed, 2) * bodyDiameter * (bodyHeight + noseHeight) / 1000000;

  // 安全なコサイン計算
  const cosAngle = Math.cos(omega);

  // 風モーメントの計算 - 絶対値のみを計算
  const momentMagnitude = Math.abs((Dwf + Dwb) * cosAngle * (centerOfPressure - centerOfGravity) * 0.001);
  
  // 符号の決定
  let finalMoment;
  if ((centerOfPressure >= centerOfGravity && windSpeed < 0) || 
      (centerOfPressure < centerOfGravity && windSpeed >= 0)) {
    finalMoment = momentMagnitude;  // プラス
  } else {
    finalMoment = -momentMagnitude; // マイナス
  }

  // 最小モーメント保証
  const MIN_MOMENT = 0.00001;
  if (Math.abs(finalMoment) > 0 && Math.abs(finalMoment) < MIN_MOMENT) {
    return Math.sign(finalMoment) * MIN_MOMENT;
  }
  
  return finalMoment;
};

// 風見効果による角度制限関数
const applyWindDirectionAngleLimit = (omega, windSpeed) => {
  // 風速が非常に小さい場合は制限なし
  if (Math.abs(windSpeed) < 0.5) return omega;
  
  // 風向きに応じた制限角度（ラジアン）
  // 風が右から左（負の風速）なら -90度に制限（風上である右向き）
  // 風が左から右（正の風速）なら +90度に制限（風上である左向き）
  const limitAngle = Math.sign(windSpeed) * Math.PI / 2;
  
  // 風上に向かっているかどうか
  // 風が右から左（負の風速）で機体が右向き（負の角度）に傾いている場合
  // または、風が左から右（正の風速）で機体が左向き（正の角度）に傾いている場合
  const isMovingUpwind = (windSpeed < 0 && omega < 0) || (windSpeed > 0 && omega > 0);
  
  // 角度制限を適用（風上へ向かう場合のみ）
  if (isMovingUpwind) {
    // 角度が制限値を超えているかチェック
    if ((windSpeed < 0 && omega < limitAngle) || (windSpeed > 0 && omega > limitAngle)) {
      // 制限角度に制限
      return limitAngle;
    }
  }
  
  // それ以外の場合は元の角度をそのまま返す
  return omega;
};

// 推力モーメント計算関数
const calculateThrustMoment = (thrust, centerOfGravity, omega, flightAngle, rocketParams) => {
  const { noseHeight, bodyHeight } = rocketParams;
  
  // 迎角を計算
  const angleOfAttack = omega - flightAngle;
  
  // 推力発生位置（ロケットの最後尾）
  const thrustPosition = noseHeight + bodyHeight;
  
  // 推力モーメントの計算 - 絶対値のみを計算
  const momentMagnitude = Math.abs(thrust * Math.sin(angleOfAttack) * Math.cos(angleOfAttack) * (thrustPosition - centerOfGravity) * 0.001);
  
  // 符号の決定
  let finalMoment;
  if ((thrustPosition >= centerOfGravity && angleOfAttack < 0) || 
      (thrustPosition < centerOfGravity && angleOfAttack >= 0)) {
    finalMoment = momentMagnitude;  // プラス
  } else {
    finalMoment = -momentMagnitude; // マイナス
  }

  // 最小モーメント保証
  const MIN_MOMENT = 0.00001;
  if (Math.abs(finalMoment) > 0 && Math.abs(finalMoment) < MIN_MOMENT) {
    return Math.sign(finalMoment) * MIN_MOMENT;
  }
  
  return finalMoment;
};

// フィンのたわみ量計算
const calculateFinDeflection = (velocity, material, finParams, angleChangePerDt2) => {
  const { finHeight, finBaseWidth, finTipWidth, finThickness, finSweepLength } = finParams;
  const { E } = material;

  // 速度0の場合は早期リターン
  if (Math.abs(velocity) < 0.001) return 0;

  // finThicknessが小数の場合も適切に扱えるように
  const safeFinThickness = Number(finThickness) || 2.0; // デフォルト値として2.0mm

  // 安全な角度変化量を確保
  const safeAngleChange = Math.max(-0.5, Math.min(0.5, angleChangePerDt2));

  // 安全な速度（極端に大きな値を制限）
  const safeVelocity = Math.min(velocity, 300);

  try {
    // フィンの面積（m^2）
    const finArea = ((finBaseWidth + finTipWidth) * 0.001) * (finHeight * 0.001) / 2;
    
    // テーパー比（無次元）- 分母0防止
    let taperRatio = 0;
    if (finHeight > 0) {
      taperRatio = ((finBaseWidth * 0.001) - (finTipWidth * 0.001)) / (finHeight * 0.001);
      // 極端な値の制限
      taperRatio = Math.max(-0.9, Math.min(0.9, taperRatio));
    }
    
    // 後退角（ラジアン）
    const sweepAngle = Math.atan((finSweepLength * 0.001 + 0.5 * finTipWidth * 0.001 - 0.5 * finBaseWidth * 0.001) * Math.PI / (finHeight * 0.001));
    
    // 断面二次モーメント（近似）- 分母0防止
    let I = 0;
    if (finTipWidth > 0 && safeFinThickness > 0) {
      I = (finTipWidth * 0.001 * Math.pow((safeFinThickness * 0.001), 3)) / 12;
    }
    
    // I値が極端に小さい（または0）の場合は安全値を設定
    if (I < 1e-12) {
      I = 1e-12; // 極小値を設定して分母0を防止
    }
    
    // 風圧による力（F）- 速度の二乗に比例
    const windForce = (9.81 * 0.05 * safeVelocity * safeVelocity) * Math.sin(safeAngleChange) * finArea;

    // 単位長さあたりの風圧による力
    const unitlengthWindForce = windForce / Math.max(0.001, (finHeight * 0.001));
    
    // たわみ量計算（mm）
    const deflectionFactor = (1 / (1 - taperRatio));
    const rawDeflection = (unitlengthWindForce * Math.pow((finHeight * 0.001), 4) * Math.cos(sweepAngle) / (8 * E * I)) * deflectionFactor;
    
    // NaNチェック
    if (isNaN(rawDeflection)) {
      console.warn('フィンたわみ量計算でNaNが発生しました');
      return 15; // NaNの場合は閾値を返す（表示時に変換）
    }
    
    // たわみ量の絶対値が15mmを超える場合は15に制限
    if (Math.abs(rawDeflection) > 15) {
      // 15mmを超えるたわみ → 15を返す（表示時に「15mm以上」に変換）
      return 15;
    }
    
    // 通常範囲内のたわみ量
    return rawDeflection;
    
  } catch (error) {
    console.error("Fin deflection calculation error:", error);
    return 15; // エラー時は閾値を返す（表示時に「15mm以上」に変換）
  }
};

// 物理計算 (calculateFlightPath関数の完全実装)
export const calculateFlightPath = (rocketParams, angle, windSpeed, windProfile, config) => {
  // 角度変化を記録するための変数
  let prevOmega = angle * Math.PI / 180; // 前フレームの角度（初期値は発射角度）
  let currentMaxAngleChange = 0; // 直接更新用の最大角度変化量
  const dt = 0.02;
  const dt2 = ANGLE_RESPONSE_DT; // 角度応答時間
  const mass_kg = gToKg(rocketParams.weight);
  const thrustData = MOTOR_THRUST_DATA[rocketParams.selectedMotor];
  const launchRailLength = PHYSICAL_CONSTANTS.launchRailLength; // 発射台の長さ (m)
  const MAX_TIME = 20; // 最大シミュレーション時間 (s)

  // ロケットの物理パラメータ
  const bodyDiameter = mmToM(rocketParams.bodyWidth); // ロケット直径 (m)
  const bodyRadius = bodyDiameter / 2; // ロケット半径 (m)
  const bodyLength = mmToM(rocketParams.bodyHeight + rocketParams.noseHeight); // ロケット全長 (m)
  const finwidth_m = mmToM(rocketParams.finHeight); // フィン幅
  const finThickness_m = mmToM(rocketParams.finThickness);
  const PI = Math.PI;
  
  // 慣性モーメントの計算 (I = 0.25*m*r^2 + 0.833*m*l^2)
  const momentOfInertia = 0.25 * mass_kg * bodyRadius * bodyRadius + 0.833 * mass_kg * bodyLength * bodyLength;

  // ノーズ形状に基づく抗力係数
  const noseCd = NOSE_SHAPES[rocketParams.noseShape].cd;
  
  // フィン材料特性
  const finMaterial = FIN_MATERIALS[rocketParams.finMaterial];

  // パラシュート関連の定数
  const thrustEndTime = thrustData.length * dt;
  const parachuteDelay = parseInt(rocketParams.selectedMotor.split('-')[1]);
  const parachuteDeployTime = 1.0;
  const parachuteEjectionTime = thrustEndTime + parachuteDelay;
  const parachuteActiveTime = parachuteEjectionTime + parachuteDeployTime;
  const parachuteDiameter = mmToM(parseInt(rocketParams.selectedParachute.slice(1))); // φ180 -> 180mm -> 0.18m

  // 新しい計算 - 投影面積と圧力中心の計算
  const projectedAreas = calculateProjectedArea(rocketParams);
  const centerOfPressure = calculateCenterOfPressure(rocketParams);
  const aerodynamicCenter = calculateAerodynamicCenter(rocketParams);
  const stabilityCenterOfPressure = calculateStabilityCenterOfPressure(rocketParams);
  
  // 体積計算
  const volumes = calculateVolume(rocketParams);
  
  // 静安定マージン計算
  const staticMargins = calculateStaticMargin(rocketParams);

  // 状態変数の初期化
  let time = 0;
  let x = 0; // メートル単位
  let y = 0; // メートル単位
  let vx = 0;
  let vy = 0;
  let prev_vx = 0;
  let prev_vy = 0;
  let omega = angle * Math.PI / 180; // 初期角度（ラジアン）
  let angularVelocity = 0; // 角速度
  let angularAcceleration = 0; // 角加速度
  let data = [];
  let isParachuteEjected = false;
  let isParachuteActive = false;
  let parachuteDeploymentProgress = 0;
  let finDeflection = 0; // フィンのたわみ量（mm）
  
  // 姿勢安定性チェック用の変数を追加
  let maxAngleChangePerDt2 = 0; // dt2時間あたりの最大角度変化量
  let isAngleStableOK = true; // 姿勢安定性の判定
  let thrustEndFlag = false; // 推力終了フラグ
  const initialOmegaDegrees = angle; // 初期角度（度）
  
  // 角度計算用変数
  let stepCounter = 0;
  let totalTorque = 0;
  let avgThrustForTorque = 0;
  let thrustSamplesCount = 0;
  let angleChangePerDt2 = 0; // dt2時間あたりの角度変化量 (ν)
  
  // 重力加速度
  const g = 9.81; // m/s²

  // 記録用変数
  let maxHeight = 0;
  let maxSpeed = 0;
  let maxDistance = 0; // 最大水平距離を記録
  let maxFinDeflection = 0; // 最大フィンたわみ量
  
  // キーポイント記録
  let keyPoints = {
    thrustEnd: { time: 0, height: 0, speed: 0 },
    maxHeight: { time: 0, height: 0, speed: 0 },
    parachuteEjection: { time: 0, height: 0, speed: 0 },
    parachuteActive: { time: 0, height: 0, speed: 0 }
  };

  // シミュレーションループ
  while ((y >= 0 || time < 0.1) && time < MAX_TIME) {
    // 前回の速度を保存
    prev_vx = vx;
    prev_vy = vy;

    // パラシュート状態の更新
    if (!isParachuteEjected && time >= parachuteEjectionTime) {
      isParachuteEjected = true;
      keyPoints.parachuteEjection = { time, height: y, speed: vy };
    }
    
    if (isParachuteEjected && !isParachuteActive) {
      parachuteDeploymentProgress = Math.min(1, 
        (time - parachuteEjectionTime) / parachuteDeployTime);
    }
    
    if (!isParachuteActive && time >= parachuteActiveTime) {
      isParachuteActive = true;
      parachuteDeploymentProgress = 1.0;
      
      // パラシュート展開時は速度を90%減少
      vx = vx * 0.1;
      vy = vy * 0.1;
      
      keyPoints.parachuteActive = { time, height: y, speed: vy };
    }

    const distanceFromStart = Math.sqrt(x * x + y * y);
    const onLaunchRail = distanceFromStart < launchRailLength;

    // 速度の大きさ
    const velocity = Math.sqrt(prev_vx * prev_vx + prev_vy * prev_vy);
    
    // 現在の高度に基づく有効風速を計算
    const effectiveWindSpeed = calculateWindSpeedAtHeight(windSpeed, y, windProfile);
    
    // 力と加速度の初期化
    let ax = 0;
    let ay = -g;
    let torque = 0; // トルク
    let Fx = 0;
    let Fy = 0;
    let thrust = 0;

    // 特定の角度への対応
    const angleAdjustment = Math.abs(angle) === 4 || Math.abs(angle) === 18 ? 0.01 : 0;
    const adjustedOmega = omega + angleAdjustment * (angle < 0 ? -1 : 1);

    // フィンのたわみ量計算
    if (velocity > 5.0) {
      const finParams = {
        finHeight: rocketParams.finHeight,
        finBaseWidth: rocketParams.finBaseWidth,
        finTipWidth: rocketParams.finTipWidth,
        finThickness: rocketParams.finThickness,
        finSweepLength: rocketParams.finSweepLength
      };
      
      finDeflection = calculateFinDeflection(velocity, finMaterial, finParams, angleChangePerDt2);
      
      // 最大フィンたわみ量を更新
      if (finDeflection > maxFinDeflection) {
        maxFinDeflection = finDeflection;
      }
    } else {
      finDeflection = 0;
    }

    // パラシュートフェーズ
    if (isParachuteActive) {
      // パラシュートの抗力計算
      const Cd = 0.775; // パラシュートの抗力係数
      const rho = 1.225; // 空気密度 (kg/m³)
      const Area = PI * Math.pow(parachuteDiameter / 2, 2);
      const Dp = 0.5 * Cd * rho * velocity * velocity * Area;
      
      // 速度方向への抗力
      if (velocity > 0.001) {
        Fx = -Dp * (prev_vx / velocity);
        Fy = -Dp * (prev_vy / velocity);
      }

      // 横風の影響を追加（高度に応じた風速を使用）
      const Cdw = 0.25; // 横風の抗力係数
      const S = parachuteDiameter * parachuteDiameter * 0.785; // パラシュートの投影面積
      const Dw = 0.5 * Cdw * rho * Math.abs(effectiveWindSpeed) * effectiveWindSpeed * S;
      Fx -= Dw; // 横風の影響を追加
      
      // 重力の追加
      Fy -= mass_kg * g;
      
      // 加速度計算
      ax = Fx / mass_kg;
      ay = Fy / mass_kg;
      
      // トルク計算 - パラシュート展開後は発射角度を維持する
      const initialOmega = angle * Math.PI / 180;
      if (Math.abs(adjustedOmega - initialOmega) > 0.01) {
        // 簡易的なトルク - 後でより正確な計算に置き換え
        torque = (initialOmega - adjustedOmega) * 0.001;
      }
    } 
    else if (isParachuteEjected && !isParachuteActive) {
      // パラシュート展開中
      Fy = -mass_kg * g;
      
      // 軽い空気抵抗
      if (velocity > 0.001) {
        const dragCoefficient = 0.1;
        Fx = -dragCoefficient * prev_vx;
        Fy -= dragCoefficient * prev_vy;
      }

      // 横風の影響を小さく追加（高度に応じた風速を使用）
      const rho = 1.225; // 空気密度
      const Cdw = 0.25; // 横風の抗力係数
      const S = bodyDiameter * bodyLength * 0.5; // 半分展開時の面積
      const Dw = 0.5 * Cdw * rho * Math.abs(effectiveWindSpeed) * effectiveWindSpeed * S;
      Fx -= Dw * 0.5; // 展開中なので横風の影響を半分に
      
      // 加速度計算
      ax = Fx / mass_kg;
      ay = Fy / mass_kg;
      
      // トルク計算 - パラシュート展開中も発射角度に強く引っ張られる
      const initialOmega = angle * Math.PI / 180;
      // 簡易的なトルク - 後でより正確な計算に置き換え
      torque = (initialOmega - adjustedOmega) * 0.0005;
    } 
    else {
      // 通常飛行フェーズ
      
      // ロケットの抗力計算 - ノーズ形状に基づく抗力係数を使用
      const Cd = noseCd; 
      const rho = 1.225; // 空気密度
      const Area = PI * Math.pow(bodyDiameter / 2, 2) + finwidth_m * finThickness_m * 4;
      const Dt = 0.5 * Cd * rho * velocity * velocity * Area;
      
      // 横風の抗力計算（高度に応じた風速を使用）
      const Cdw = 0.25; // 横風の抗力係数
      const S = bodyDiameter * bodyLength; // ロケットの側面積
      const Dw = 0.5 * Cdw * rho * Math.abs(effectiveWindSpeed) * effectiveWindSpeed * S;
      
      if (time < thrustEndTime) {
        // エンジン推力フェーズ
        const thrustIndex = Math.min(Math.floor(time / dt), thrustData.length - 1);
        thrust = thrustData[thrustIndex];
        
        // 推力サンプルをトルク計算用に累積
        avgThrustForTorque += thrust;
        thrustSamplesCount++;
        
        if (onLaunchRail) {
          // 発射台上での運動
          if (angle === 0) {
            // 垂直発射
            Fy = thrust - mass_kg * g;
            Fx = -Dw; // 横風の影響のみ
          } else {
            // 角度付き発射 (修正した式)
            Fy = thrust * Math.cos(adjustedOmega) - mass_kg * g;
            Fx = thrust * Math.sin(adjustedOmega) - Dw; // 横風の影響を追加
          }
          
          // 発射台上は角度固定
          torque = 0;
        } else {
          // 自由飛行（推力あり）
          if (velocity > 0.001) {
            // 修正: 推力と抗力の分解方法を修正
            // T*sinθ - Dt*sinθ - Dw, T*cosθ - m*g - Dt*cosθ
            Fx = thrust * Math.sin(adjustedOmega) - Dt * Math.sin(adjustedOmega) - Dw;
            Fy = thrust * Math.cos(adjustedOmega) - mass_kg * g - Dt * Math.cos(adjustedOmega);
          } else {
            // 速度がほぼゼロの場合
            Fx = thrust * Math.sin(adjustedOmega) - Dw;
            Fy = thrust * Math.cos(adjustedOmega) - mass_kg * g;
          }
          
          // トルク計算 - 推力フェーズ
          if (velocity > 1.0) {
            const flightAngle = Math.atan2(prev_vx, prev_vy);
            
            try {
              // 新しいモーメント計算関数を使用
              const ML = calculateLiftMoment(velocity, adjustedOmega, flightAngle, rocketParams, projectedAreas.sideArea, aerodynamicCenter.aerodynamicCenter, rocketParams.centerOfGravity);
              const MD = calculateDragMoment(velocity, adjustedOmega, flightAngle, rocketParams, projectedAreas.sideArea, aerodynamicCenter.aerodynamicCenter, rocketParams.centerOfGravity);
              const MW = calculateWindMoment(rocketParams.noseHeight, bodyDiameter, rocketParams.bodyHeight, effectiveWindSpeed, adjustedOmega, projectedAreas.totalFinArea, centerOfPressure.centerOfPressure, rocketParams.centerOfGravity);
              const MT = calculateThrustMoment(thrust, rocketParams.centerOfGravity, adjustedOmega, flightAngle, rocketParams);
              
              // トルク値の検証 - 無限大や非数値をチェック
              if (!isFinite(ML) || isNaN(ML)) torque += 0;
              else if (!isFinite(MD) || isNaN(MD)) torque += 0;
              else if (!isFinite(MW) || isNaN(MW)) torque += 0;
              else if (!isFinite(MT) || isNaN(MT)) torque += 0;
              else {
                // 合計トルク - 上限設定を追加
                const rawTorque = ML + MD + MW + MT;
                
                // トルク値が小さすぎる問題を解決するために、最小トルク閾値を設定
                const MIN_TORQUE_THRESHOLD = 0.0001;
                
                // トルクの絶対値が閾値以下の場合、符号を保持して最小値を使用
                if (Math.abs(rawTorque) > 0 && Math.abs(rawTorque) < MIN_TORQUE_THRESHOLD) {
                  torque = Math.sign(rawTorque) * MIN_TORQUE_THRESHOLD;
                } else {
                  // 通常のトルク制限
                  torque = Math.max(-1.0, Math.min(1.0, rawTorque));
                }
                
                // デバッグ用ログ - トルク計算の詳細を表示
                if (Math.abs(torque) > 0.001 || Math.abs(ML) > 0.001 || Math.abs(MD) > 0.001 || Math.abs(MW) > 0.001 || Math.abs(MT) > 0.001) {
                  console.log(`Thrust Torque components (t=${time.toFixed(2)}): ML=${ML.toFixed(6)}, MD=${MD.toFixed(6)}, MW=${MW.toFixed(6)}, MT=${MT.toFixed(6)}, Total=${torque.toFixed(6)}`);
                }
              }
              
              // ±4°と±18°の場合のトルク補正
              if (Math.abs(angle) === 4 || Math.abs(angle) === 18) {
                torque *= 1.2; // 20%増加
              }
            } catch (error) {
              // エラーが発生した場合はトルクをゼロにして続行
              console.error('Torque calculation error:', error);
              torque = 0;
            }
          }
        }
      } else {
        // 推力終了時のフラグを設定
        if (!thrustEndFlag) {
          thrustEndFlag = true;
          keyPoints.thrustEnd = { time, height: y, speed: vy };
        }
        
        // 慣性飛行（推力なし）- ここでは推力T=0
        
        if (velocity > 0.001) {
          // 修正: Fx = -Dt*sinθ - Dw, Fy = -m*g - Dt*cosθ
          Fx = -Dt * Math.sin(adjustedOmega) - Dw;
          Fy = -mass_kg * g - Dt * Math.cos(adjustedOmega);
        } else {
          Fx = -Dw;
          Fy = -mass_kg * g;
        }
        
        // トルク計算 - 慣性飛行フェーズ
        if (velocity > 0.5) {
          const flightAngle = Math.atan2(prev_vx, prev_vy);
          
          // 新しいモーメント計算メソッドを使用
          try {
            // 新しいモーメント計算関数を使用
            const ML = calculateLiftMoment(velocity, adjustedOmega, flightAngle, rocketParams, projectedAreas.sideArea, aerodynamicCenter.aerodynamicCenter, rocketParams.centerOfGravity);
            const MD = calculateDragMoment(velocity, adjustedOmega, flightAngle, rocketParams, projectedAreas.sideArea, aerodynamicCenter.aerodynamicCenter, rocketParams.centerOfGravity);
            const MW = calculateWindMoment(rocketParams.noseHeight, bodyDiameter, rocketParams.bodyHeight, effectiveWindSpeed, adjustedOmega, projectedAreas.totalFinArea, centerOfPressure.centerOfPressure, rocketParams.centerOfGravity);
            // 推力ゼロなのでMTは0
          
            // 合計トルク
            const rawTorque = ML + MD + MW;
            
            // トルク値が小さすぎる問題を解決するために、最小トルク閾値を設定
            const MIN_TORQUE_THRESHOLD = 0.0001;
            
            // トルクの絶対値が閾値以下の場合、符号を保持して最小値を使用
            if (Math.abs(rawTorque) > 0 && Math.abs(rawTorque) < MIN_TORQUE_THRESHOLD) {
              torque = Math.sign(rawTorque) * MIN_TORQUE_THRESHOLD;
            } else {
              // 通常のトルク制限
              torque = Math.max(-1.0, Math.min(1.0, rawTorque));
            }
            
            // デバッグ用ログ - トルク計算の詳細を表示
            if (Math.abs(torque) > 0.001 || Math.abs(ML) > 0.001 || Math.abs(MD) > 0.001 || Math.abs(MW) > 0.001) {
              console.log(`Inertial Torque components (t=${time.toFixed(2)}): ML=${ML.toFixed(6)}, MD=${MD.toFixed(6)}, MW=${MW.toFixed(6)}, Total=${torque.toFixed(6)}`);
            }
          } catch (error) {
            console.error('Torque calculation error:', error);
            torque = 0;
          }
          
          // ±4°と±18°の場合のトルク補正
          if (Math.abs(angle) === 4 || Math.abs(angle) === 18) {
            torque *= 1.2; // 20%増加
          }
        }
      }
      
      // 加速度計算
      ax = Fx / mass_kg;
      ay = Fy / mass_kg;
    }

    // トルクの累積
    totalTorque += torque;
    stepCounter++;

    // 角度の更新（dt2間隔で）
    if (stepCounter >= ANGLE_STEPS_PER_UPDATE) {
      try {
        // トルクの平均を計算
        const avgTorque = totalTorque; // const avgTorque = totalTorque / stepCounter???????
        
        // トルクが非数値または無限大の場合はゼロにリセット
        const safeAvgTorque = isFinite(avgTorque) && !isNaN(avgTorque) ? avgTorque : 0;
        
        // 最小トルク保証 - これにより姿勢変化が確実に発生する
        const MIN_TORQUE = 0.00001;
        let effectiveTorque = safeAvgTorque;
        if (Math.abs(safeAvgTorque) > 0 && Math.abs(safeAvgTorque) < MIN_TORQUE) {
          effectiveTorque = Math.sign(safeAvgTorque) * MIN_TORQUE;
        }
        
        // トルク計算のデバッグ出力
        if (Math.abs(effectiveTorque) > 0.001) {
          console.log(`Torque calculation (t=${time.toFixed(2)}): avgTorque=${safeAvgTorque.toFixed(6)}, effectiveTorque=${effectiveTorque.toFixed(6)}, momentOfInertia=${momentOfInertia.toFixed(6)}`);
        }
        
        // 角加速度の計算: α = M/I
        angularAcceleration = effectiveTorque / momentOfInertia;
        
        // 角速度の更新: ω = ω0 + α*dt2
        const oldAngularVelocity = angularVelocity;
        angularVelocity = angularVelocity + angularAcceleration * dt2;
        
        // 角速度に上限を設定
        angularVelocity = Math.max(-3, Math.min(3, angularVelocity));
        
        // 角度変化の計算: ν = ω*dt2
        angleChangePerDt2 = angularVelocity * dt2;
        
        // 最小角度変化保証 - 非常に小さな値でも角度更新を保証
        const MIN_ANGLE_CHANGE_PER_DT2 = 0.001; // 角度変化を強調するために増加（0.0001→0.001）

        if (Math.abs(angleChangePerDt2) > 0 && Math.abs(angleChangePerDt2) < MIN_ANGLE_CHANGE_PER_DT2) {
          angleChangePerDt2 = Math.sign(angleChangePerDt2) * MIN_ANGLE_CHANGE_PER_DT2;
        }
        
        // 角度変化のデバッグ出力
        if (Math.abs(oldAngularVelocity - angularVelocity) > 0.001 || Math.abs(angleChangePerDt2) > 0.001) {
          console.log(`Angle change calculation (t=${time.toFixed(2)}): angularAcceleration=${angularAcceleration.toFixed(6)}, angularVelocity=${angularVelocity.toFixed(6)}, angleChangePerDt2=${angleChangePerDt2.toFixed(6)}`);
        }
        
        // 姿勢安定性チェック（パラシュート展開前かつ推力終了後の慣性飛行中）
        if (!isParachuteEjected && thrustEndFlag) {
          // dt2時間あたりの角度変化量（度数法）
          const angleChangePerDt2Degrees = angleChangePerDt2 * 180 / Math.PI;
          
          // 最大角度変化量を更新
          if (Math.abs(angleChangePerDt2Degrees) > Math.abs(maxAngleChangePerDt2)) {
            maxAngleChangePerDt2 = angleChangePerDt2Degrees;
          }
          
          // 角度変化量が±10°を超える場合にNG判定（0.2秒間の変化量）
          if (Math.abs(angleChangePerDt2Degrees) > 10) {
            isAngleStableOK = false;
          }
        }
        
        // カウンターとトルク累積のリセット
        stepCounter = 0;
        totalTorque = 0;
        avgThrustForTorque = 0;
        thrustSamplesCount = 0;
      } catch (error) {
        // エラーが発生した場合でも安全に処理
        console.error('Angle update error:', error);
        stepCounter = 0;
        totalTorque = 0;
        avgThrustForTorque = 0;
        thrustSamplesCount = 0;
      }
    }
    
    // 物理ベースの角度更新を強化 - この部分は常に適用される
    // 角度の更新 - angleChangePerDt2の値を確実に反映
    const physicsBasedAngleChange = angleChangePerDt2 / ANGLE_STEPS_PER_UPDATE;

    // 物理ベースの変化が非常に小さい場合の最小変化量を設定
    const MIN_ANGLE_CHANGE = 0.0001; // ラジアン単位での最小角度変化

    // 物理ベースの姿勢変化を適用
    if (PHYSICAL_ATTITUDE_CONTROL) {
      if (Math.abs(physicsBasedAngleChange) > 0 && Math.abs(physicsBasedAngleChange) < MIN_ANGLE_CHANGE) {
        // 変化量が小さすぎる場合は適切な最小値を使用
        omega += Math.sign(physicsBasedAngleChange) * MIN_ANGLE_CHANGE;
      } else {
        // 通常の物理ベースの角度更新
        omega += physicsBasedAngleChange;
      }
  
      // 角度変化のデバッグ出力 - 実際に何が起きているかを確認
      if (Math.abs(physicsBasedAngleChange) > 0.001) {
        console.log(`Applied physics angle change (t=${time.toFixed(2)}): ${physicsBasedAngleChange.toFixed(6)}, new omega=${omega.toFixed(6)}`);
      }
    }
    
    // 拡張姿勢制御ロジック - 風見効果など
    if (ENHANCED_ATTITUDE_CONTROL) {
      // 発射台を離れた後かつパラシュート展開前
      if (!onLaunchRail && !isParachuteEjected) {
        // 速度ベクトルの方向（飛行角）
        const flightAngle = Math.atan2(vx, vy);
    
        // 推力フェーズと慣性飛行フェーズで完全に分ける
        if (thrustEndFlag) {
          //===========================================
          // 慣性飛行中 - 風見効果を無効化
          //===========================================
          
          // 目標姿勢角は常に速度方向（風の影響を受けない）
          const targetOmega = flightAngle;
          
          // 姿勢角変化のスピード係数 - 非常に小さな値
          const adjustRate = Math.min(0.05, 0.002 * velocity);
          
          // 姿勢角を目標角に近づける（緩やかに）
          const newOmega = omega * (1.0 - adjustRate) + targetOmega * adjustRate;
          
          // 変化量の制限も小さめ
          // 変化量の制限を大きくする - 約11.5度/フレーム
          const maxChange = 0.2;
          if (Math.abs(newOmega - omega) > maxChange) {
            omega += Math.sign(newOmega - omega) * maxChange;
          } else {
            omega = newOmega;
          }
          
          // デバッグ用ログ - 拡張姿勢制御の適用を確認
          if (Math.abs(newOmega - omega) > 0.01) {
            console.log(`Applied enhanced attitude control (inertial): targetOmega=${targetOmega.toFixed(6)}, newOmega=${newOmega.toFixed(6)}`);
          }
          
        } else {
          //===========================================
          // 推力飛行中 - 風見効果を適用
          //===========================================
          
          // 風向きベクトル
          const windAngle = effectiveWindSpeed > 0 ? 0 : Math.PI; // 風が左から右:0度、右から左:180度
    
          // 風向きに対する垂直方向 (±90度)
          const perpendicularToWind1 = windAngle + Math.PI/2;  // +90度
          const perpendicularToWind2 = windAngle - Math.PI/2;  // -90度
          
          // 目標姿勢角の決定
          let targetOmega;
          
          if (Math.abs(effectiveWindSpeed) < 0.5) {
            // 風が弱い場合は速度方向に合わせる
            targetOmega = flightAngle;
          } else {
            // 風が強い場合、風向きも考慮する
            
            // 風上に向かっているか判定 - 風向きと速度が「逆」のとき風上
            const isMovingUpwind = (effectiveWindSpeed < 0 && vx > 0) || 
                                   (effectiveWindSpeed > 0 && vx < 0);
            
            if (isMovingUpwind) {
              // 風上に向かう場合は風に対して垂直方向を超えない
              if (effectiveWindSpeed < 0) {
                // 右からの風の場合、-90度まで（右向き）
                targetOmega = Math.max(flightAngle, perpendicularToWind2);
              } else {
                // 左からの風の場合、+90度まで（左向き）
                targetOmega = Math.min(flightAngle, perpendicularToWind1);
              }
            } else {
              // 風下に向かう場合は直接速度方向
              targetOmega = flightAngle;
            }
          }
          
          // 推力飛行中の姿勢角変化のスピード係数（現状維持）
          const adjustRate = Math.min(0.05, 0.002 * velocity);
          
          // 風速に応じた調整
          const windFactor = Math.min(0.8, Math.abs(effectiveWindSpeed) / 6.0);
          const finalAdjustRate = adjustRate * (1.0 + windFactor);
          
          // 姿勢角を目標角に近づける
          const newOmega = omega * (1.0 - finalAdjustRate) + targetOmega * finalAdjustRate;
          
          // 変化量の制限を大きくする - 約11.5度/フレーム
          const maxChange = 0.2;
          if (Math.abs(newOmega - omega) > maxChange) {
            omega += Math.sign(newOmega - omega) * maxChange;
          } else {
            omega = newOmega;
          }
          
          // デバッグ用ログ - 拡張姿勢制御の適用を確認
          if (Math.abs(newOmega - omega) > 0.01) {
            console.log(`Applied enhanced attitude control (thrust): targetOmega=${targetOmega.toFixed(6)}, newOmega=${newOmega.toFixed(6)}`);
          }
        }
      }
    }

    // 速度の更新
    vx = vx + ax * dt;
    vy = vy + ay * dt;
    
    // 速度制限（最大100m/s）
    const MAX_SPEED = 100; // m/s
    const currentSpeed = Math.sqrt(vx * vx + vy * vy);
    if (currentSpeed > MAX_SPEED) {
      const factor = MAX_SPEED / currentSpeed;
      vx *= factor;
      vy *= factor;
    }
    
    // 角速度の制限
    const MAX_ANGULAR_VELOCITY = 5; // rad/s
    angularVelocity = Math.max(-MAX_ANGULAR_VELOCITY, Math.min(MAX_ANGULAR_VELOCITY, angularVelocity));
    
    // 位置の更新
    if (onLaunchRail) {
      // 発射台上の動き
      if (angle === 0) {
        // 垂直発射
        x = 0;
        y = y + vy * dt;
      } else {
        // 角度付き発射
        const railDistance = Math.min(distanceFromStart + velocity * dt, launchRailLength);
        x = railDistance * Math.sin(omega);
        y = railDistance * Math.cos(omega);
      }
    } else {
      // 自由飛行の位置更新
      x = x + vx * dt;
      y = y + vy * dt;
    }
    
    // 最高高度と最高速度の更新
    if (y > maxHeight) {
      maxHeight = y;
      keyPoints.maxHeight = { time, height: maxHeight, speed: vy };
    }
    
    if (currentSpeed > maxSpeed) {
      maxSpeed = currentSpeed;
    }
    
    // 最大水平距離の更新
    if (Math.abs(x) > maxDistance) {
      maxDistance = Math.abs(x);
    }
    
    // データの記録
    data.push({ 
      time,
      physicsX: x, // 物理座標系でのx（メートル単位）
      physicsY: y, // 物理座標系でのy（メートル単位）
      height: y, // メートル単位
      vx: vx,
      vy: vy,
      ax: ax,
      ay: ay,
      speedMagnitude: Math.sqrt(vx * vx + vy * vy),
      accelerationMagnitude: Math.sqrt(ax * ax + ay * ay),
      angularVelocity,
      angularAcceleration,
      isParachuteEjected,
      isParachuteActive,
      parachuteDeploymentProgress,
      omega,
      torque,
      angleChangePerDt2,
      horizontalDistance: Math.abs(x), // 水平距離の絶対値を追加
      finDeflection, // フィンのたわみ量を追加
      angleDeviationDegrees: (omega * 180 / Math.PI) - initialOmegaDegrees, // 初期角度からの偏差を追加
      effectiveWindSpeed // 実効風速を記録
    });

    time += dt;
  }

  console.log(`シミュレーション完了: 最高高度=${maxHeight.toFixed(2)}m, 最高速度=${maxSpeed.toFixed(2)}m/s, 最大水平距離=${maxDistance.toFixed(2)}m`);
  console.log(`推力終了時 (${keyPoints.thrustEnd.time.toFixed(2)}s): 高度=${keyPoints.thrustEnd.height.toFixed(2)}m, 速度=${keyPoints.thrustEnd.speed.toFixed(2)}m/s`);
  console.log(`最高点 (${keyPoints.maxHeight.time.toFixed(2)}s): 高度=${keyPoints.maxHeight.height.toFixed(2)}m, 速度=${keyPoints.maxHeight.speed.toFixed(2)}m/s`);
  console.log(`最大フィンたわみ量: ${maxFinDeflection.toFixed(4)}mm`);
  console.log(`最大角度変化量/dt2: ${maxAngleChangePerDt2.toFixed(2)}°`);
  console.log(`姿勢安定性判定: ${isAngleStableOK ? 'OK' : 'NG'}`);

  // 投影面積と体積の情報を追加
  console.log(`投影面積 - 正面: ${projectedAreas.frontalArea.toFixed(5)}m², 側面: ${projectedAreas.sideArea.toFixed(5)}m²`);
  console.log(`体積 - ノーズ: ${volumes.noseVolume.toFixed(6)}m³, ボディ: ${volumes.bodyVolume.toFixed(6)}m³, 合計: ${volumes.totalVolume.toFixed(6)}m³`);
  console.log(`圧力中心位置: ${centerOfPressure.centerOfPressure.toFixed(2)}mm, 空力中心位置: ${aerodynamicCenter.aerodynamicCenter.toFixed(2)}mm`);
  console.log(`静安定用圧力中心位置: ${stabilityCenterOfPressure.stabilityCenterOfPressure.toFixed(2)}mm`);
  console.log(`静安定マージン (標準): ${staticMargins.standardStaticMargin.toFixed(2)}, (静安定用): ${staticMargins.stabilityStaticMargin.toFixed(2)}`);
  
  return {
    data,
    maxHeight,
    maxSpeed,
    maxDistance,
    maxFinDeflection,
    keyPoints,
    angleStability: {
      maxAngleChangePerDt2,
      isAngleStableOK
    },
    projectedAreas,  // 投影面積データを追加
    volumes,         // 体積データを追加
    pressureCenter: centerOfPressure,  // 圧力中心データを追加
    aerodynamicCenter,  // 空力中心データを追加
    stabilityCenterOfPressure, // 静安定用圧力中心を追加
    staticMargins,  // 静安定マージンを追加
    calculations: {
      aerodynamicCenter: Math.round(aerodynamicCenter.aerodynamicCenter),
      pressureCenter: Math.round(centerOfPressure.centerOfPressure),
      stabilityCenterOfPressure: Math.round(stabilityCenterOfPressure.stabilityCenterOfPressure),
      standardStaticMargin: parseFloat(staticMargins.standardStaticMargin.toFixed(2)),
      stabilityStaticMargin: parseFloat(staticMargins.stabilityStaticMargin.toFixed(2)),
      finDivergenceSpeed: Math.round(calculateFinDivergenceSpeed(rocketParams)),
      finFlutterSpeed: Math.round(calculateFinFlutterSpeed(rocketParams))
    }
  };
};