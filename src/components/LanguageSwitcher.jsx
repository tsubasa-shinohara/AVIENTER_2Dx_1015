import React from 'react';
import { useTranslation } from 'react-i18next';

const LanguageSwitcher = () => {
  const { i18n } = useTranslation();

  const changeLanguage = (lng) => {
    i18n.changeLanguage(lng);
  };

  return (
    <div className="flex gap-2">
      <button
        onClick={() => changeLanguage('ja')}
        className={`px-3 py-1 rounded text-sm font-medium transition-colors ${
          i18n.language === 'ja'
            ? 'bg-blue-500 text-white'
            : 'bg-gray-200 text-gray-700 hover:bg-gray-300'
        }`}
      >
        日本語
      </button>
      <button
        onClick={() => changeLanguage('en')}
        className={`px-3 py-1 rounded text-sm font-medium transition-colors ${
          i18n.language === 'en'
            ? 'bg-blue-500 text-white'
            : 'bg-gray-200 text-gray-700 hover:bg-gray-300'
        }`}
      >
        English
      </button>
    </div>
  );
};

export default LanguageSwitcher;
