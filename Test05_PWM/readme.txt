サンプルプログラム（main.c）では、TIM2のCH1、TIM16のCH1をそれぞれ同じduty比で光らせています。

CubeMx側でPWMOutputの設定を行い、プリスケーラとピリオドを設定すれば問題なく動くはずです。

動作クロックとプリスケーラ、ピリオドによって出力する周波数が変化します。

コード中では
プリスケーラ:uhPrescalerValue(uint32_t型グローバル変数定義)
ピリオド:PERIOD_VALUE(#define定義)
動作クロック：SystemCoreClock（システム定義）<-定数
として定義されています。

計算式：
