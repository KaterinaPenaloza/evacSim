package evacSim.agents;

import javax.swing.*;
import java.awt.*;

public class LoadingDialog {
    private static JWindow splash;
    
    public static void show() {
        if (splash != null) return;
        
        splash = new JWindow();
        splash.setAlwaysOnTop(true);
        
        JPanel content = new JPanel(new BorderLayout(10, 10));
        content.setBorder(BorderFactory.createCompoundBorder(
            BorderFactory.createLineBorder(new Color(70, 130, 180), 3),
            BorderFactory.createEmptyBorder(30, 40, 30, 40)
        ));
        content.setBackground(Color.WHITE);
        
        JLabel message = new JLabel(
            "<html><div style='text-align: center;'>" +
            "<b style='font-size: 18px; color: #4682B4;'>Inicializando EvacSim</b><br><br>" +
            "<span style='font-size: 14px;'>Cargando datos y configuración...</span><br>" +
            "<span style='font-size: 12px; color: #666;'>Por favor espere</span>" +
            "</div></html>"
        );
        message.setHorizontalAlignment(JLabel.CENTER);
        content.add(message, BorderLayout.CENTER);
        
        JProgressBar progressBar = new JProgressBar();
        progressBar.setIndeterminate(true);
        progressBar.setPreferredSize(new Dimension(320, 22));
        progressBar.setForeground(new Color(70, 130, 180));
        content.add(progressBar, BorderLayout.SOUTH);
        
        splash.setContentPane(content);
        splash.setSize(400, 180);
        splash.setLocationRelativeTo(null);
        
        // CRÍTICO: Forzar que se dibuje AHORA
        splash.setVisible(true);
        splash.toFront();
        splash.repaint();
    }
    
    public static void close() {
        if (splash != null) {
            splash.setVisible(false);
            splash.dispose();
            splash = null;
        }
    }
}