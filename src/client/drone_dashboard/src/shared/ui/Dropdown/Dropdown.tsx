import React, { useState } from 'react';

interface DropdownProps {
    title: string | React.ReactNode; // Заголовок блока
    children: React.ReactNode;      // Содержимое, которое будет раскрываться
}

export const Dropdown: React.FC<DropdownProps> = ({ title, children }) => {
    const [isOpen, setIsOpen] = useState(false);

    const toggleOpen = () => setIsOpen(!isOpen);

    return (
        <div style={{ marginBottom: '10px' }}>
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
                {title}
            </div>
            {isOpen && (
                <div style={{ marginTop: '10px', paddingLeft: '20px' }}>
                    {children}
                </div>
            )}
        </div>
    );
};