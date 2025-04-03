import React from 'react';
import Plot from 'react-plotly.js';

interface PlotlyChartComponentProps {
    data: Array<{ x: string[]; y: number[]; name: string }>;
    title?: string;
}

export const PlotlyChartComponent: React.FC<PlotlyChartComponentProps> = ({ data, title }) => {
    return (
        <Plot
            data={data}
            layout={{
                title,
                xaxis: { title: 'Time' },
                yaxis: { title: 'Value' },
                showlegend: true, // Показываем легенду
            }}
            config={{ displayModeBar: false }}
            style={{ width: '100%', height: '400px' }}
            useResizeHandler={true} // Автоматически изменять размер графика
        />
    );
};