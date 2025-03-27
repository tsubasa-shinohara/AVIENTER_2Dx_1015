// SVG描画関連の関数
import React from 'react';
import { SVG_CONFIG, ANALYSIS_VIEW_CONFIG } from './RocketConstants';

// 安全な値を取得するユーティリティ関数 - より強化されたバージョン
export const getSafeValue = (value, defaultValue = 0) => {
  // 値がnull、undefined、NaN、無限大の場合はデフォルト値を返す
  if (value === null || value === undefined || isNaN(value) || !isFinite(value)) {
    return defaultValue;
  }
  // 数値型でない場合、変換を試みる
  if (typeof value !== 'number') {
    const parsed = parseFloat(value);
    // 変換できなければデフォルト値を返す
    return isNaN(parsed) ? defaultValue : parsed;
  }
  return value;
};

// getNosePath 関数の修正
export const getNosePath = (config, noseShape, noseHeight, bodyWidth, bodyHeight) => {
  const startX = config.centerX - bodyWidth / 2;
  const endX = config.centerX + bodyWidth / 2;
  const baseY = config.height - bodyHeight; // ノーズコーンの底辺
  const topY = baseY - noseHeight; // ノーズコーンの先端

  if (noseShape === "cone") {
    // 円錐ノーズ
    return `M ${startX} ${baseY} L ${config.centerX} ${topY} L ${endX} ${baseY} Z`;
  } else if (noseShape === "parabola") {
    // 放物線ノーズ
    const controlY = topY + noseHeight * 0.15;
    return `M ${startX} ${baseY} 
            C ${startX} ${controlY}, ${config.centerX} ${topY}, ${config.centerX} ${topY}
            C ${config.centerX} ${topY}, ${endX} ${controlY}, ${endX} ${baseY} Z`;
  } else {
    // オジブノーズ
    const halfWidth = bodyWidth / 2;
    return `M ${startX} ${baseY}
            Q ${startX + halfWidth * 0.1} ${topY + noseHeight * 0.4}, ${config.centerX} ${topY}
            Q ${endX - halfWidth * 0.1} ${topY + noseHeight * 0.4}, ${endX} ${baseY} Z`;
  }
};

// getBodyPath 関数の修正
export const getBodyPath = (config, bodyHeight, bodyWidth) => {
  const startX = config.centerX - bodyWidth / 2;
  const endX = config.centerX + bodyWidth / 2;
  const baseY = config.height;
  const topY = baseY - bodyHeight;

  return `M ${startX} ${baseY} 
          L ${startX} ${topY} 
          L ${endX} ${topY} 
          L ${endX} ${baseY} Z`;
};

// getLeftFinPath 関数の修正 - 後退代（finSweepLength）がマイナスの場合に対応
export const getLeftFinPath = (config, bodyWidth, finHeight, finBaseWidth, finSweepLength, finTipWidth) => {
  const bodyLeftX = config.centerX - bodyWidth / 2;
  const finBottomY = config.height;
  const finTopY = finBottomY - finBaseWidth;

  // 後退代がマイナスの場合の処理を追加
  let adjustedFinTopY, adjustedFinBottomY;
  
  if (finSweepLength >= 0) {
    // 通常の正の後退代の場合
    adjustedFinTopY = finTopY + finSweepLength;
    adjustedFinBottomY = adjustedFinTopY + finTipWidth;
  } else {
    // 後退代がマイナスの場合（前進翼）
    // finBaseWidthに|finSweepLength|を足す
    adjustedFinTopY = finTopY - Math.abs(finSweepLength);
    adjustedFinBottomY = adjustedFinTopY + finTipWidth;
  }

  return `M ${bodyLeftX} ${finTopY}
          L ${bodyLeftX} ${finBottomY}
          L ${bodyLeftX - finHeight} ${adjustedFinBottomY}
          L ${bodyLeftX - finHeight} ${adjustedFinTopY} Z`;
};

// getRightFinPath 関数の修正 - 後退代（finSweepLength）がマイナスの場合に対応
export const getRightFinPath = (config, bodyWidth, finHeight, finBaseWidth, finSweepLength, finTipWidth) => {
  const bodyRightX = config.centerX + bodyWidth / 2;
  const finBottomY = config.height;
  const finTopY = finBottomY - finBaseWidth;

  // 後退代がマイナスの場合の処理を追加
  let adjustedFinTopY, adjustedFinBottomY;
  
  if (finSweepLength >= 0) {
    // 通常の正の後退代の場合
    adjustedFinTopY = finTopY + finSweepLength;
    adjustedFinBottomY = adjustedFinTopY + finTipWidth;
  } else {
    // 後退代がマイナスの場合（前進翼）
    adjustedFinTopY = finTopY - Math.abs(finSweepLength);
    adjustedFinBottomY = adjustedFinTopY + finTipWidth;
  }

  return `M ${bodyRightX} ${finTopY}
          L ${bodyRightX} ${finBottomY}
          L ${bodyRightX + finHeight} ${adjustedFinBottomY}
          L ${bodyRightX + finHeight} ${adjustedFinTopY} Z`;
};

// getCenterFinsPath 関数の修正 - 後退代（finSweepLength）がマイナスの場合に対応
export const getCenterFinsPath = (config, finThickness, finBaseWidth, finSweepLength, finTipWidth) => {
  const halfThickness = finThickness / 2;
  const finBottomY = config.height;
  const finTopY = finBottomY - finBaseWidth;

  // 後退代がマイナスの場合の処理を追加
  let finExtension;
  
  if (finSweepLength >= 0) {
    // 通常の正の後退代の場合 - フィンの後端が左右のフィンと揃うように調整
    finExtension = Math.max(0, finSweepLength + finTipWidth - finBaseWidth);
  } else {
    // 後退代がマイナスの場合（前進翼）
    // finBaseWidthに|finSweepLength|を足し、フィンの前端位置を調整
    finExtension = Math.max(0, finTipWidth - (finBaseWidth + Math.abs(finSweepLength)));
  }

  // 後退代がマイナスの場合、中央フィンの前端も調整する
  let frontExtension = 0;
  if (finSweepLength < 0) {
    frontExtension = Math.abs(finSweepLength);
  }

  return `M ${config.centerX - halfThickness} ${finTopY - frontExtension}
          L ${config.centerX - halfThickness} ${finBottomY}
          L ${config.centerX - halfThickness} ${finBottomY + finExtension}
          L ${config.centerX + halfThickness} ${finBottomY + finExtension}
          L ${config.centerX + halfThickness} ${finBottomY}
          L ${config.centerX + halfThickness} ${finTopY - frontExtension} Z`;
};

// 3枚フィン用の左右のフィンを描画する関数
export const getTriFinLeftRightPaths = (config, bodyWidth, finHeight, finBaseWidth, finSweepLength, finTipWidth) => {
  const centerX = config.centerX;
  const bottomY = config.height;
  
  // ボディの半径
  const bodyRadius = bodyWidth / 2;
  
  // フィンの付け根の位置: ボディの中心から ±(ボディ径/2×1.73/2) の位置
  const offset = bodyRadius * 1.73 / 2;
  const leftFinRootX = centerX - offset;
  const rightFinRootX = centerX + offset;
  
  // フィンの幅: 設定値 × 1.73 / 2
  const adjustedFinHeight = finHeight * 1.73 / 2;
  
  // フィン付け根の上端位置
  const finTopY = bottomY - finBaseWidth;
  
  // 後退代がマイナスの場合の処理を追加
  let adjustedFinTopY, adjustedFinBottomY;
  
  if (finSweepLength >= 0) {
    // 通常の正の後退代の場合
    adjustedFinTopY = finTopY + finSweepLength;
    adjustedFinBottomY = adjustedFinTopY + finTipWidth;
  } else {
    // 後退代がマイナスの場合（前進翼）
    adjustedFinTopY = finTopY - Math.abs(finSweepLength);
    adjustedFinBottomY = adjustedFinTopY + finTipWidth;
  }
  
  // 左フィンのパス
  const leftFinPath = `M ${leftFinRootX} ${finTopY}
                       L ${leftFinRootX} ${bottomY}
                       L ${leftFinRootX - adjustedFinHeight} ${adjustedFinBottomY}
                       L ${leftFinRootX - adjustedFinHeight} ${adjustedFinTopY} Z`;
  
  // 右フィンのパス
  const rightFinPath = `M ${rightFinRootX} ${finTopY}
                        L ${rightFinRootX} ${bottomY}
                        L ${rightFinRootX + adjustedFinHeight} ${adjustedFinBottomY}
                        L ${rightFinRootX + adjustedFinHeight} ${adjustedFinTopY} Z`;
  
  return {
    leftFin: leftFinPath,
    rightFin: rightFinPath
  };
};

// 姿勢表示用に中央のフィンを描画する関数
export const getCenterFinsPathForAttitude = (x, y, params) => {
  // configが有効かどうかを確認
  if (!config || typeof config !== 'object' || !('centerX' in config) || !('height' in config)) {
    console.warn('getCenterFinsPathForAttitude: Invalid config object', config);
    // デフォルト設定を作成
    config = { centerX: 400, height: 550 };
  }

  // 安全なパラメータ処理
  finThickness = getSafeValue(finThickness, 2);
  finBaseWidth = getSafeValue(finBaseWidth, 30);
  finSweepLength = getSafeValue(finSweepLength, 0);
  finTipWidth = getSafeValue(finTipWidth, 20);

  const halfThickness = params.finThickness / 2;
  const finBottomY = y;
  const finTopY = finBottomY - params.finBaseWidth;
  
  // フィンの翼端位置を正確に計算
  const tipTopY = finTopY + params.finSweepLength;
  const tipBottomY = tipTopY + params.finTipWidth;
  
  // フィンの後端が左右のフィンと揃うように調整
  // フィンの翼端長がボディ後端よりも伸びる場合を計算
  const finExtension = Math.max(0, params.finSweepLength + params.finTipWidth - params.finBaseWidth);
  
  return `M ${x - halfThickness} ${finTopY}
          L ${x - halfThickness} ${finBottomY}
          L ${x - halfThickness} ${finBottomY + finExtension}
          L ${x + halfThickness} ${finBottomY + finExtension}
          L ${x + halfThickness} ${finBottomY}
          L ${x + halfThickness} ${finTopY} Z`;
};

// シミュレーションタブ用のロケット描画関数を追加
export const drawInitialRocket = (centerX, centerY, params, rotation) => {
    // 回転を考慮した変換
    const transform = `translate(${centerX}, ${centerY}) rotate(${rotation})`;
    
    // 各部品のパス計算
    const bodyPath = `M ${-params.bodyWidth / 2} ${-params.bodyHeight} 
                     L ${-params.bodyWidth / 2} ${0} 
                     L ${params.bodyWidth / 2} ${0} 
                     L ${params.bodyWidth / 2} ${-params.bodyHeight} Z`;
                     
    let nosePath;
    if (params.noseShape === "cone") {
      nosePath = `M ${-params.bodyWidth / 2} ${-params.bodyHeight} 
                  L ${0} ${-params.bodyHeight - params.noseHeight} 
                  L ${params.bodyWidth / 2} ${-params.bodyHeight} Z`;
    } else if (params.noseShape === "parabola") {
      nosePath = `M ${-params.bodyWidth / 2} ${-params.bodyHeight} 
                  Q ${0} ${-params.bodyHeight - params.noseHeight * 1.2}, 
                    ${params.bodyWidth / 2} ${-params.bodyHeight} Z`;
    } else { // ogive
      nosePath = `M ${-params.bodyWidth / 2} ${-params.bodyHeight} 
                  Q ${0} ${-params.bodyHeight - params.noseHeight * 1.1}, 
                    ${params.bodyWidth / 2} ${-params.bodyHeight} Z`;
    }
    
    const leftFinPath = `M ${-params.bodyWidth / 2} ${-params.finBaseWidth} 
                         L ${-params.bodyWidth / 2} ${0} 
                         L ${-params.bodyWidth / 2 - params.finHeight} ${params.finSweepLength} 
                         L ${-params.bodyWidth / 2 - params.finHeight} ${params.finSweepLength - params.finTipWidth} Z`;
    
    const rightFinPath = `M ${params.bodyWidth / 2} ${-params.finBaseWidth} 
                          L ${params.bodyWidth / 2} ${0} 
                          L ${params.bodyWidth / 2 + params.finHeight} ${params.finSweepLength} 
                          L ${params.bodyWidth / 2 + params.finHeight} ${params.finSweepLength - params.finTipWidth} Z`;
    
    return {
      transform,
      bodyPath,
      nosePath,
      leftFinPath,
      rightFinPath,
      centerFinsPath: `M ${-params.finThickness / 2} ${-params.finBaseWidth} 
                      L ${-params.finThickness / 2} ${0} 
                      L ${params.finThickness / 2} ${0} 
                      L ${params.finThickness / 2} ${-params.finBaseWidth} Z`
    };
  };

// フィンの集合を取得する関数の修正 - 後退代（finSweepLength）がマイナスの場合に対応
export const getFinPaths = (x, y, params, deflection = 0) => {
  // 安全にパラメータを取得
  const bodyWidth = getSafeValue(params.bodyWidth, 30);
  const finHeight = getSafeValue(params.finHeight, 40);
  const finBaseWidth = getSafeValue(params.finBaseWidth, 30);
  const finSweepLength = getSafeValue(params.finSweepLength, 0);
  const finTipWidth = getSafeValue(params.finTipWidth, 20);
  const finThickness = getSafeValue(params.finThickness, 2);
  
  const bodyLeftX = x - bodyWidth / 2;
  const bodyRightX = x + bodyWidth / 2;
  const finBottomY = y;
  const finTopY = finBottomY - finBaseWidth;
  
  // たわみを考慮した修正
  const deflectionAngle = deflection * Math.PI / 180; // ラジアンに変換
  
  // 後退代がマイナスの場合の処理を追加
  let adjustedFinTopY, adjustedFinBottomY;
  
  if (finSweepLength >= 0) {
    // 通常の正の後退代の場合
    adjustedFinTopY = finTopY + finSweepLength;
    adjustedFinBottomY = adjustedFinTopY + finTipWidth;
  } else {
    // 後退代がマイナスの場合（前進翼）
    adjustedFinTopY = finTopY - Math.abs(finSweepLength);
    adjustedFinBottomY = adjustedFinTopY + finTipWidth;
  }
  
  // たわみを適用したフィン座標
  const leftFinTipX = bodyLeftX - finHeight * Math.cos(deflectionAngle);
  const leftFinTipY = adjustedFinTopY + finHeight * Math.sin(deflectionAngle);
  const leftFinBottomTipX = bodyLeftX - finHeight * Math.cos(deflectionAngle);
  const leftFinBottomTipY = adjustedFinBottomY + finHeight * Math.sin(deflectionAngle);
  
  const rightFinTipX = bodyRightX + finHeight * Math.cos(deflectionAngle);
  const rightFinTipY = adjustedFinTopY + finHeight * Math.sin(deflectionAngle);
  const rightFinBottomTipX = bodyRightX + finHeight * Math.cos(deflectionAngle);
  const rightFinBottomTipY = adjustedFinBottomY + finHeight * Math.sin(deflectionAngle);

  const leftFin = `M ${bodyLeftX} ${finTopY}
                  L ${bodyLeftX} ${finBottomY}
                  L ${leftFinBottomTipX} ${leftFinBottomTipY}
                  L ${leftFinTipX} ${leftFinTipY} Z`;

  const rightFin = `M ${bodyRightX} ${finTopY}
                  L ${bodyRightX} ${finBottomY}
                  L ${rightFinBottomTipX} ${rightFinBottomTipY}
                  L ${rightFinTipX} ${rightFinTipY} Z`;

  // 中央フィンの描画 - 後退代がマイナスの場合の処理を追加
  const halfThickness = finThickness / 2;
  let frontExtension = 0;
  if (finSweepLength < 0) {
    frontExtension = Math.abs(finSweepLength);
  }
  
  const centerFins = `M ${x - halfThickness} ${finTopY - frontExtension}
                    L ${x - halfThickness} ${finBottomY}
                    L ${x + halfThickness} ${finBottomY}
                    L ${x + halfThickness} ${finTopY - frontExtension} Z`;

  return { leftFin, rightFin, centerFins };
};

// 風向き矢印の描画関数
export const getWindArrow = (windSpeed) => {
  if (windSpeed === 0) return null;
  
  // 風速に基づいた矢印の長さ
  const arrowLength = Math.min(100, Math.max(40, Math.abs(windSpeed) * 10));
  
  // 「風速: X m/s」テキストの中央位置
  const textCenter = 85; // テキスト中央のx座標
  const y = 50; // テキスト下のy座標
  
  // 矢印の色と太さを取得
  const color = getWindArrowColor(windSpeed);
  const strokeWidth = getWindArrowWidth(windSpeed);
  
  // 矢印の頭の大きさ
  const headSize = 15;
  
  // 風速の符号で場合分け
  if (windSpeed < 0) {
    // マイナスの場合 - 右向き矢印
    const startX = textCenter - arrowLength / 2;
    const endX = textCenter + arrowLength / 2;
    
    return {
      line: `M ${startX} ${y} L ${endX} ${y}`,
      head: `M ${endX - headSize} ${y - headSize/2} L ${endX} ${y} L ${endX - headSize} ${y + headSize/2}`,
      y: y,
      color: color,
      strokeWidth: strokeWidth
    };
  } else {
    // プラスの場合 - 左向き矢印だが、矢じりは右端
    const lineStart = textCenter - arrowLength / 2;
    const lineEnd = textCenter + arrowLength / 2;
    
    return {
      line: `M ${lineStart} ${y} L ${lineEnd} ${y}`,
      head: `M ${lineStart + headSize} ${y - headSize/2} L ${lineStart} ${y} L ${lineStart + headSize} ${y + headSize/2}`,
      y: y,
      color: color,
      strokeWidth: strokeWidth
    };
  }
};

// 風速に応じた矢印の色を取得する関数
export const getWindArrowColor = (speed) => {
  const absSpeed = Math.abs(speed);
  if (absSpeed >= 6) return "#e74c3c"; // 強風: 赤色
  if (absSpeed >= 3) return "#f39c12"; // 中風: オレンジ/黄色
  return "#2ecc71"; // 弱風: 緑色
};

// 風速に応じた矢印の線の太さを取得する関数
export const getWindArrowWidth = (speed) => {
  const absSpeed = Math.abs(speed);
  if (absSpeed >= 6) return 5; // 強風: より太い線
  if (absSpeed >= 3) return 4; // 中風: やや太い線
  return 3; // 弱風: 通常の線
};

// パラシュート表示を改良
export const getParachutePath = (x, y, isOpen, deploymentProgress = 1.0, rotation = 0, rocketLength, selectedParachute, trajectoryScale) => {
  if (!isOpen) return "";

  // ロケットの長さを基準に計算
  // 基準距離を短くして調整（1.5倍にする）
  const baseOffsetDistance = rocketLength * 1.5; 
  const offsetDistance = baseOffsetDistance * trajectoryScale * deploymentProgress;

  // 機体の上部に常に配置するため、回転角度を無視
  // xは変更せず、yだけ上方向に移動
  const parachuteX = x;
  const parachuteY = y - offsetDistance;

  // パラシュートサイズの計算
  const parachuteSize = (parseInt(selectedParachute.slice(1)) / 1000 / 2) * trajectoryScale * deploymentProgress;

  // パラシュートのパスを描画
  return `M ${parachuteX - parachuteSize} ${parachuteY}
          C ${parachuteX - parachuteSize} ${parachuteY - parachuteSize * 0.8},
            ${parachuteX} ${parachuteY - parachuteSize * 1.2},
            ${parachuteX + parachuteSize} ${parachuteY - parachuteSize * 0.8}
          C ${parachuteX + parachuteSize} ${parachuteY - parachuteSize * 0.8},
            ${parachuteX + parachuteSize} ${parachuteY},
            ${parachuteX + parachuteSize} ${parachuteY}
          Z`;
};

// パラシュートの紐を描画
export const getParachuteStringPaths = (x, y, isOpen, deploymentProgress = 1.0, rotation = 0, rocketLength, selectedParachute, trajectoryScale) => {
  if (!isOpen) return '';

  // パラシュート本体と全く同じ計算を使用して一貫性を確保
  const baseOffsetDistance = rocketLength * 1.5; 
  const offsetDistance = baseOffsetDistance * trajectoryScale * deploymentProgress;

  // 機体の上部に常に配置
  const parachuteX = x;
  const parachuteY = y - offsetDistance;

  // パラシュートサイズも同一の計算で
  const parachuteSize = (parseInt(selectedParachute.slice(1)) / 1000 / 2) * trajectoryScale * deploymentProgress;

  // キャノピーの各部分の座標を計算
  const canopyLeftX = parachuteX - parachuteSize;
  const canopyRightX = parachuteX + parachuteSize;
  const canopyBottomY = parachuteY;
  const canopyCenterX = parachuteX;
  const canopyCenterTopY = parachuteY - parachuteSize * 0.5;

  // 紐のパスを描画 - ロケット位置からキャノピーの各点まで
  return `M ${x} ${y} L ${canopyLeftX} ${canopyBottomY}
          M ${x} ${y} L ${canopyCenterX} ${canopyCenterTopY}
          M ${x} ${y} L ${canopyRightX} ${canopyBottomY}`;
};

// メートル単位からSVG座標への変換関数
export const metersToSvgX = (meters, trajectoryScale) => {
  // NaNチェック追加
  if (isNaN(meters)) {
    console.warn('metersToSvgX: 無効なメートル値', meters);
    return SVG_CONFIG.centerX;
  }
  
  // trajectoryScaleが数値でない場合のフォールバック
  if (typeof trajectoryScale !== 'number' || isNaN(trajectoryScale) || trajectoryScale <= 0) {
    console.warn('metersToSvgX: 無効なtrajectoryScale', trajectoryScale);
    trajectoryScale = 10.0; // より安全なデフォルト値
  }
  
  return SVG_CONFIG.centerX + meters * trajectoryScale;
};

export const metersToSvgY = (meters, trajectoryScale) => {
  // NaNチェック
  if (isNaN(meters)) {
    console.warn('metersToSvgY: 無効なメートル値', meters);
    return SVG_CONFIG.groundLevel;
  }
  
  // trajectoryScaleが数値でない場合のフォールバック
  if (typeof trajectoryScale !== 'number' || isNaN(trajectoryScale) || trajectoryScale <= 0) {
    console.warn('metersToSvgY: 無効なtrajectoryScale', trajectoryScale);
    trajectoryScale = 10.0; // より安全なデフォルト値
  }
  
  return SVG_CONFIG.groundLevel - meters * trajectoryScale;
};

// 安全な回転属性を生成する関数
export const getSafeRotationTransform = (rotation, x, y) => {
  // 全ての値にNaNチェック
  const safeRotation = isNaN(rotation) ? 0 : rotation;
  const safeX = isNaN(x) ? 400 : x;
  const safeY = isNaN(y) ? 550 : y;

  return `rotate(${safeRotation} ${safeX} ${safeY})`;
};

// 姿勢表示用のロケットスケールを自動計算する関数を追加
export const calculateAttitudeDisplayScale = (rocketParams, circleRadius = 90) => {
  try {
    // 必要なパラメータを安全に取得
    const noseHeight = getSafeValue(rocketParams.noseHeight, 50);
    const bodyHeight = getSafeValue(rocketParams.bodyHeight, 200);
    const finSweepLength = getSafeValue(rocketParams.finSweepLength, 0);
    const finTipWidth = getSafeValue(rocketParams.finTipWidth, 20);
    const finBaseWidth = getSafeValue(rocketParams.finBaseWidth, 30);
    
    // ロケットの寸法を計算
    const actualRocketHeight = noseHeight + bodyHeight;
    const finExtension = Math.max(0, finSweepLength + finTipWidth - finBaseWidth);
    const totalHeight = actualRocketHeight + finExtension;
    
    // スケール計算 - 安全範囲内に制限
    const rawScale = (circleRadius * 1.6) / Math.max(50, totalHeight);
    return Math.max(0.01, Math.min(2.0, rawScale));
  } catch (error) {
    console.error('calculateAttitudeDisplayScale エラー:', error);
    return 0.3; // エラー時のデフォルト値
  }
};

// 視覚化のための風速高度分布矢印を描画する関数
export const getWindProfileArrows = (baseWindSpeed, profile, showWindArrows, metersToSvgY, calculateWindSpeedAtHeight) => {
  if (!showWindArrows) return [];
  
  // 表示する高度レベル（メートル）
  const heights = [0, 20, 40, 60, 80];
  
  return heights.map(height => {
    // 各高度での風速を計算
    const windSpeed = calculateWindSpeedAtHeight(baseWindSpeed, height, profile);
    
    // SVG座標に変換
    const y = metersToSvgY(height);
    
    // 風速によって矢印の長さを制限して表示範囲内に収める
    // 最大長さをより小さく設定し、範囲内に確実に収める
    const maxArrowLength = 60; // 最大長さを短くして確実に範囲内に収める
    const minArrowLength = 20; // 最小長さ
    const arrowLength = Math.min(maxArrowLength, Math.max(minArrowLength, Math.abs(windSpeed) * 6));
    
    const color = getWindArrowColor(windSpeed);
    const strokeWidth = getWindArrowWidth(windSpeed);
    
    // 表示位置の調整 - 右に寄せて表示するように修正
    const startX = 80; // 右寄りに配置
    // 矢印の向きを修正 - 既存の風向き矢印と同じロジック
    const direction = windSpeed < 0 ? 1 : -1;  // 負の風速で右向き、正の風速で左向き
    
    // 矢印の端点の位置を計算
    const endX = startX + arrowLength * direction;
    
    // テキスト位置の調整
    // 矢印の長さと方向に応じてテキスト位置を調整し、矢印から適切な距離に配置
    const textOffsetX = direction > 0 ? 12 : -35; // テキストのオフセット
    
    return {
      height,
      windSpeed,
      y,
      arrowPath: {
        // 矢印の線
        line: `M ${startX} ${y} L ${endX} ${y}`,
        // 矢印の頭
        head: direction > 0 
          ? `M ${endX - 10} ${y - 5} L ${endX} ${y} L ${endX - 10} ${y + 5}`
          : `M ${endX + 10} ${y - 5} L ${endX} ${y} L ${endX + 10} ${y + 5}`,
        // 風速テキスト - 常に表示範囲内に収まるように位置調整
        textX: endX + textOffsetX,
        textY: y,
        color,
        strokeWidth
      }
    };
  });
};

// ロケット寸法計算関数
export const getActualRocketDimensions = (config, noseHeight, bodyHeight, finSweepLength, finTipWidth, finBaseWidth) => {
  const rocketTopY = config.height - bodyHeight - noseHeight;
  // 機体全長の計算 - ノーズとボディの高さ
  const actualRocketHeight = noseHeight + bodyHeight;
  // フィンの後端がボディ後端よりも出る場合の計算
  const finExtension = Math.max(0, finSweepLength + finTipWidth - finBaseWidth);
  // 全長は機体長 + フィン後端部分
  const totalHeight = actualRocketHeight + finExtension;

  return { rocketTopY, actualRocketHeight, totalHeight };
};

// 設計タブ用のviewBox計算関数
export const getDesignViewBox = (design, noseHeight, bodyHeight, finHeight, finSweepLength, finTipWidth, finBaseWidth, bodyWidth) => {
  // 直接ロケット寸法を計算
  const rocketTopY = design.height - bodyHeight - noseHeight;
  const actualRocketHeight = noseHeight + bodyHeight;
  const finExtension = Math.max(0, finSweepLength + finTipWidth - finBaseWidth);
  const totalHeight = actualRocketHeight + finExtension;
  
  // フィンの左右の最大幅を計算
  const maxFinWidth = finHeight;

  // 左右のマージンを追加 - より広いマージンを確保
  const horizontalMargin = Math.max(maxFinWidth + 20, design.width * 0.25);
  
  // 垂直方向のマージンを追加 - 新規追加
  const verticalMargin = Math.max(totalHeight * 0.2, 50);
  
  // 全幅 (ボディ幅 + 左右のフィン)
  const totalWidth = bodyWidth + (maxFinWidth * 2);
  
  // 余裕を持たせた表示エリアのサイズ - 余裕を増やす
  const paddedHeight = totalHeight * 1.4; // 40%の余裕に増加
  const paddedWidth = totalWidth * 1.4;  // 40%の余裕に増加
  
  // 表示エリアのアスペクト比
  const viewBoxAspectRatio = design.width / design.height;
  
  // ロケットのアスペクト比と比較して表示方法を決定
  let viewBoxWidth, viewBoxHeight;
  
  if (paddedWidth / paddedHeight > viewBoxAspectRatio) {
    // 幅が制限要素の場合
    viewBoxWidth = paddedWidth;
    viewBoxHeight = paddedWidth / viewBoxAspectRatio;
  } else {
    // 高さが制限要素の場合
    viewBoxHeight = paddedHeight;
    viewBoxWidth = paddedHeight * viewBoxAspectRatio;
  }
  
  // 表示領域を計算 - 上部に余裕を持たせるように変更
  return `-${horizontalMargin} -${verticalMargin} ${design.width + horizontalMargin * 2} ${design.height + verticalMargin * 2}`;
};

// 重量・空力特性タブ用のviewBox計算関数（改善版）
export const getAnalysisViewBox = () => {
    // ロケットが確実に中央に表示されるようなviewBox
    return "-200 -400 400 800";
  };

// ロケットの拡大率を計算する関数
export const calculateRocketScale = (totalHeight, canvasHeight) => {
  // 画面の70%を使うように計算
  const targetHeight = canvasHeight * 0.7;
  
  // スケール係数の計算
  const scale = targetHeight / totalHeight;
  
  // 最小・最大スケール値の制限
  return Math.max(0.5, Math.min(2.0, scale));
};

// ロケットのSVG位置変換を計算する関数
export const getRocketTransform = (totalHeight, canvasWidth, canvasHeight) => {
  // 動的なスケール計算
  const scale = calculateRocketScale(totalHeight, canvasHeight);
  
  // 移動量の計算（キャンバスの中央に配置）
  const translateX = canvasWidth / 2;
  const translateY = canvasHeight / 2 - (totalHeight * scale / 2);
  
  return {
    translateX,
    translateY,
    scale,
    transform: `translate(${translateX}, ${translateY}) scale(${scale})`
  };
};