import React, { useState } from 'react';
import { mmToM } from './RocketConstants';

const ExportTab = ({ rocketSim }) => {
  const [isPremiumUser, setIsPremiumUser] = useState(false);
  const [showUpgradeMessage, setShowUpgradeMessage] = useState(false);

  const generateFinSVG = () => {
    const {
      finHeight,
      finBaseWidth,
      finTipWidth,
      finSweepLength,
      finThickness,
      finCount
    } = rocketSim;

    const scale = 1;
    const padding = 20;
    
    let adjustedFinTopY, adjustedFinBottomY;
    
    if (finSweepLength >= 0) {
      adjustedFinTopY = finSweepLength;
      adjustedFinBottomY = adjustedFinTopY + finTipWidth;
    } else {
      adjustedFinTopY = -Math.abs(finSweepLength);
      adjustedFinBottomY = adjustedFinTopY + finTipWidth;
    }

    const finPath = `M 0 0
                     L 0 ${finBaseWidth}
                     L ${finHeight} ${adjustedFinBottomY}
                     L ${finHeight} ${adjustedFinTopY}
                     Z`;

    const svgWidth = (finHeight + padding * 2) * scale;
    const svgHeight = (Math.max(finBaseWidth, adjustedFinBottomY) + padding * 2) * scale;

    const svg = `<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" 
     width="${svgWidth}mm" 
     height="${svgHeight}mm" 
     viewBox="0 0 ${svgWidth} ${svgHeight}">
  <defs>
    <style>
      .fin-outline { 
        fill: none; 
        stroke: #000000; 
        stroke-width: 0.1; 
        vector-effect: non-scaling-stroke;
      }
      .dimension-line {
        fill: none;
        stroke: #0000ff;
        stroke-width: 0.05;
        stroke-dasharray: 2,2;
      }
      .dimension-text {
        font-family: Arial, sans-serif;
        font-size: 12px;
        fill: #0000ff;
      }
    </style>
  </defs>
  
  <!-- フィン形状 (レーザーカッター用カットライン) -->
  <g transform="translate(${padding * scale}, ${padding * scale}) scale(${scale})">
    <path class="fin-outline" d="${finPath}" />
  </g>
  
  <!-- 寸法線と注釈 -->
  <g class="dimension-text">
    <text x="${padding * scale / 2}" y="${svgHeight / 2}" text-anchor="middle" transform="rotate(-90, ${padding * scale / 2}, ${svgHeight / 2})">
      フィン付け根幅: ${finBaseWidth}mm
    </text>
    <text x="${svgWidth / 2}" y="${svgHeight - 5}" text-anchor="middle">
      フィン高さ: ${finHeight}mm
    </text>
    <text x="${svgWidth - 50}" y="20">
      板厚: ${finThickness}mm
    </text>
    <text x="${svgWidth - 50}" y="35">
      翼端幅: ${finTipWidth}mm
    </text>
    <text x="${svgWidth - 50}" y="50">
      後退代: ${finSweepLength}mm
    </text>
    <text x="${svgWidth - 50}" y="65">
      枚数: ${finCount}枚
    </text>
  </g>
  
  <!-- 製作情報 -->
  <text x="10" y="${svgHeight - 10}" class="dimension-text" font-size="10">
    AVIENTER 2D - Fin Template (1:1 scale)
  </text>
</svg>`;

    return svg;
  };

  const downloadSVG = () => {
    if (!isPremiumUser) {
      setShowUpgradeMessage(true);
      return;
    }

    const svgContent = generateFinSVG();
    const blob = new Blob([svgContent], { type: 'image/svg+xml' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = `rocket-fin-${Date.now()}.svg`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
  };

  const renderPreviewSVG = () => {
    const {
      finHeight,
      finBaseWidth,
      finTipWidth,
      finSweepLength,
      finThickness,
      finCount
    } = rocketSim;

    const scale = 1;
    const padding = 20;
    
    let adjustedFinTopY, adjustedFinBottomY;
    
    if (finSweepLength >= 0) {
      adjustedFinTopY = finSweepLength;
      adjustedFinBottomY = adjustedFinTopY + finTipWidth;
    } else {
      adjustedFinTopY = -Math.abs(finSweepLength);
      adjustedFinBottomY = adjustedFinTopY + finTipWidth;
    }

    const finPath = `M 0 0
                     L 0 ${finBaseWidth}
                     L ${finHeight} ${adjustedFinBottomY}
                     L ${finHeight} ${adjustedFinTopY}
                     Z`;

    const svgWidth = (finHeight + padding * 2) * scale;
    const svgHeight = (Math.max(finBaseWidth, adjustedFinBottomY) + padding * 2) * scale;

    return (
      <svg 
        xmlns="http://www.w3.org/2000/svg" 
        width={`${svgWidth}mm`}
        height={`${svgHeight}mm`}
        viewBox={`0 0 ${svgWidth} ${svgHeight}`}
        style={{ maxWidth: '100%', height: 'auto' }}
      >
        <defs>
          <style>{`
            .fin-outline { 
              fill: none; 
              stroke: #000000; 
              stroke-width: 0.1; 
              vector-effect: non-scaling-stroke;
            }
            .dimension-text {
              font-family: Arial, sans-serif;
              font-size: 12px;
              fill: #0000ff;
            }
          `}</style>
        </defs>
        
        <g transform={`translate(${padding * scale}, ${padding * scale}) scale(${scale})`}>
          <path className="fin-outline" d={finPath} />
        </g>
        
        <g className="dimension-text">
          <text 
            x={padding * scale / 2} 
            y={svgHeight / 2} 
            textAnchor="middle" 
            transform={`rotate(-90, ${padding * scale / 2}, ${svgHeight / 2})`}
          >
            フィン付け根幅: {finBaseWidth}mm
          </text>
          <text x={svgWidth / 2} y={svgHeight - 5} textAnchor="middle">
            フィン高さ: {finHeight}mm
          </text>
          <text x={svgWidth - 50} y="20">
            板厚: {finThickness}mm
          </text>
          <text x={svgWidth - 50} y="35">
            翼端幅: {finTipWidth}mm
          </text>
          <text x={svgWidth - 50} y="50">
            後退代: {finSweepLength}mm
          </text>
          <text x={svgWidth - 50} y="65">
            枚数: {finCount}枚
          </text>
        </g>
        
        <text x="10" y={svgHeight - 10} className="dimension-text" fontSize="10">
          AVIENTER 2D - Fin Template (1:1 scale)
        </text>
      </svg>
    );
  };

  const handleTogglePremium = () => {
    setIsPremiumUser(!isPremiumUser);
    if (showUpgradeMessage) {
      setShowUpgradeMessage(false);
    }
  };

  return (
    <div className="space-y-6">
      <div className="bg-white border border-gray-300 rounded-lg p-6">
        <h3 className="text-xl font-bold mb-4">形状出力</h3>
        
        {!isPremiumUser && (
          <div className="mb-4 p-4 bg-blue-50 border border-blue-200 rounded">
            <div className="flex items-start">
              <div className="flex-shrink-0">
                <svg className="h-5 w-5 text-blue-400" fill="currentColor" viewBox="0 0 20 20">
                  <path fillRule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7-4a1 1 0 11-2 0 1 1 0 012 0zM9 9a1 1 0 000 2v3a1 1 0 001 1h1a1 1 0 100-2v-3a1 1 0 00-1-1H9z" clipRule="evenodd" />
                </svg>
              </div>
              <div className="ml-3 flex-1">
                <h4 className="text-sm font-medium text-blue-800">
                  プレミアム機能
                </h4>
                <div className="mt-2 text-sm text-blue-700">
                  <p>この機能は会員限定機能です。SVGファイルをダウンロードするには、プレミアム会員へのアップグレードが必要です。</p>
                  <p className="mt-2">※現在は開発版のため、下のボタンで一時的にプレミアム機能を有効化できます。</p>
                </div>
                <button
                  onClick={handleTogglePremium}
                  className="mt-3 px-4 py-2 bg-blue-600 text-white rounded hover:bg-blue-700 transition"
                >
                  プレミアム機能を有効化（開発用）
                </button>
              </div>
            </div>
          </div>
        )}

        {isPremiumUser && (
          <div className="mb-4 p-4 bg-green-50 border border-green-200 rounded">
            <div className="flex items-start">
              <div className="flex-shrink-0">
                <svg className="h-5 w-5 text-green-400" fill="currentColor" viewBox="0 0 20 20">
                  <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clipRule="evenodd" />
                </svg>
              </div>
              <div className="ml-3 flex-1">
                <h4 className="text-sm font-medium text-green-800">
                  プレミアム会員
                </h4>
                <div className="mt-1 text-sm text-green-700">
                  <p>SVGエクスポート機能をご利用いただけます。</p>
                </div>
                <button
                  onClick={handleTogglePremium}
                  className="mt-2 text-sm text-green-600 hover:text-green-800 underline"
                >
                  無効化（開発用）
                </button>
              </div>
            </div>
          </div>
        )}

        {showUpgradeMessage && !isPremiumUser && (
          <div className="mb-4 p-4 bg-yellow-50 border border-yellow-200 rounded">
            <p className="text-sm text-yellow-800">
              プレミアム会員にアップグレードしてください。将来的には、こちらからアップグレード手続きが可能になります。
            </p>
          </div>
        )}

        <div className="space-y-4">
          <div className="bg-gray-50 p-4 rounded border border-gray-200">
            <h4 className="font-semibold mb-2">現在のフィン設計</h4>
            <div className="grid grid-cols-2 gap-2 text-sm">
              <div>フィン高さ:</div>
              <div>{rocketSim.finHeight}mm</div>
              <div>付け根幅:</div>
              <div>{rocketSim.finBaseWidth}mm</div>
              <div>翼端幅:</div>
              <div>{rocketSim.finTipWidth}mm</div>
              <div>後退代:</div>
              <div>{rocketSim.finSweepLength}mm</div>
              <div>板厚:</div>
              <div>{rocketSim.finThickness}mm</div>
              <div>フィン枚数:</div>
              <div>{rocketSim.finCount}枚</div>
            </div>
          </div>

          <div className="flex gap-4">
            <button
              onClick={downloadSVG}
              className={`flex-1 px-6 py-3 rounded font-medium transition ${
                isPremiumUser
                  ? 'bg-blue-500 text-white hover:bg-blue-600'
                  : 'bg-gray-300 text-gray-500 cursor-not-allowed'
              }`}
              disabled={!isPremiumUser}
            >
              SVGファイルをダウンロード
            </button>
          </div>

          <div className="border border-gray-300 rounded p-4 bg-white">
            <h4 className="font-semibold mb-2">プレビュー</h4>
            <div className="bg-gray-100 p-4 rounded overflow-auto max-h-96 flex justify-center items-center">
              {isPremiumUser ? (
                renderPreviewSVG()
              ) : (
                <div className="flex items-center justify-center h-64 text-gray-400">
                  <div className="text-center">
                    <svg className="mx-auto h-12 w-12 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 15v2m-6 4h12a2 2 0 002-2v-6a2 2 0 00-2-2H6a2 2 0 00-2 2v6a2 2 0 002 2zm10-10V7a4 4 0 00-8 0v4h8z" />
                    </svg>
                    <p className="mt-2">プレビューはプレミアム会員のみ利用可能です</p>
                  </div>
                </div>
              )}
            </div>
          </div>

          <div className="bg-blue-50 border border-blue-200 rounded p-4">
            <h4 className="font-semibold mb-2 text-blue-900">レーザーカッターでの使用方法</h4>
            <ol className="list-decimal list-inside space-y-1 text-sm text-blue-800">
              <li>SVGファイルをダウンロードします</li>
              <li>お使いのレーザーカッターソフトウェアでファイルを開きます</li>
              <li>材料の板厚が設計値（{rocketSim.finThickness}mm）と一致することを確認します</li>
              <li>スケールが1:1（100%）になっていることを確認します</li>
              <li>黒い線がカットラインとして認識されることを確認します</li>
              <li>必要枚数（{rocketSim.finCount}枚）をカットします</li>
            </ol>
          </div>

          <div className="bg-yellow-50 border border-yellow-200 rounded p-4">
            <h4 className="font-semibold mb-2 text-yellow-900">注意事項</h4>
            <ul className="list-disc list-inside space-y-1 text-sm text-yellow-800">
              <li>出力されたSVGファイルは1:1スケールです。印刷時やカット時にスケールを変更しないでください</li>
              <li>材料の板厚は設計値と正確に一致させてください</li>
              <li>レーザーカッターの焦点距離や出力設定は材料に応じて調整してください</li>
              <li>安全のため、レーザーカッター使用時は必ず換気を行い、保護具を着用してください</li>
            </ul>
          </div>
        </div>
      </div>
    </div>
  );
};

export default ExportTab;
