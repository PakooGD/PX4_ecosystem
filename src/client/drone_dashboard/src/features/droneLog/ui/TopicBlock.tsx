import React, { useState } from 'react';
import { Dropdown } from '../../../shared/ui/Dropdown/Dropdown';
import { TopicData } from '../../../shared/types/ITypes';

interface TopicBlockProps {
    topic: string;
    data: TopicData;
}

export const TopicBlock: React.FC<TopicBlockProps> = ({ topic, data }) => {
    const [isOpen, setIsOpen] = useState(false);

    const toggleOpen = () => setIsOpen(!isOpen);

    return (
        <div>
            <div
                onClick={toggleOpen}
                style={{
                    cursor: 'pointer',
                    padding: '10px',
                    border: '1px solid #ccc',
                    borderRadius: '5px',
                    backgroundColor: isOpen ? '#f0f0f0' : '#fff',
                }}
            >
                <strong>Topic: {topic}</strong>
            </div>
            {isOpen && (
                <div style={{ marginTop: '10px', paddingLeft: '20px' }}>
                    {Object.entries(data.data).map(([key, value]) => (
                        <div key={key}>
                            <strong>{key}:</strong> {value}
                        </div>
                    ))}
                </div>
            )}
        </div>
    );
};