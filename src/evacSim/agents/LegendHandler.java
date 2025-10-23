package evacSim.agents;

import javax.swing.*;
import repast.simphony.context.Context;
import java.awt.*;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;

public class LegendHandler {
    private static final int MAX_ATTEMPTS = 5;
    private static final int DELAY_MS = 2000;

    /**
     * Inicia el proceso para agregar la leyenda al panel de visualización.
     * @param context Contexto de la simulación (no usado directamente, pero incluido por compatibilidad)
     */
    public void addLegendToDisplay(Context context) {
        SwingUtilities.invokeLater(() -> {
            Timer timer = new Timer(DELAY_MS, null);
            final int[] attempts = {0};

            timer.addActionListener(e -> {
                attempts[0]++;
                
                if (attempts[0] > MAX_ATTEMPTS) {
                    timer.stop();
                    System.out.println("No se pudo agregar la leyenda");
                    return;
                }
                
                // Buscar el WorldWindowGLJPanel
                Window[] windows = Window.getWindows();
                for (Window window : windows) {
                    if (window instanceof JFrame && window.isVisible()) {
                        JFrame frame = (JFrame) window;
                        Component targetPanel = findComponentByClass(frame.getContentPane(), "WorldWindowGLJPanel");
                        
                        if (targetPanel != null) {
                            Container parent = targetPanel.getParent();
                            if (parent instanceof JPanel) {
                                addLegendToPanel((JPanel) parent);
                                timer.stop();
                                
                                return;
                            }
                        }
                    }
                }
                
              
            });
            
            timer.setRepeats(true);
            timer.start();
        });
    }

    /**
     * Busca un componente por su nombre de clase en el contenedor.
     * @param container Contenedor donde buscar
     * @param className Nombre de la clase del componente
     * @return Componente encontrado o null si no se encuentra
     */
    private Component findComponentByClass(Container container, String className) {
        for (Component comp : container.getComponents()) {
            if (comp.getClass().getSimpleName().equals(className)) {
                return comp;
            }
            if (comp instanceof Container) {
                Component found = findComponentByClass((Container) comp, className);
                if (found != null) return found;
            }
        }
        return null;
    }

    /**
     * Agrega la leyenda al panel especificado.
     * @param displayPanel Panel donde se agregará la leyenda
     */
    private void addLegendToPanel(JPanel displayPanel) {
        try {
            // Crear panel de leyenda con círculos de color
            JPanel legendPanel = new JPanel() {
                @Override
                protected void paintComponent(Graphics g) {
                    super.paintComponent(g);
                    Graphics2D g2d = (Graphics2D) g;
                    g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
                    
                    int x = 10;
                    int y = 25;
                    int circleSize = 14;
                    int spacing = 22;
                    
                    // Título
                    g2d.setColor(Color.BLACK);
                    g2d.setFont(new Font("Arial", Font.BOLD, 12));
                    g2d.drawString("LEYENDA", x, 15);
                    
                    // Configurar para los items
                    g2d.setFont(new Font("Arial", Font.PLAIN, 11));
                    
                    // Item 1 - 0,5 m/s (Naranja oscuro)
                    g2d.setColor(new Color(189, 94, 0));
                    g2d.fillOval(x, y - circleSize/2, circleSize, circleSize);
                    g2d.setColor(Color.BLACK);
                    g2d.drawString("0,5 m/s", x + circleSize + 8, y + 4);
                    
                    // Item 2 - 1,0 m/s (Amarillo)
                    y += spacing;
                    g2d.setColor(new Color(244, 235, 113));
                    g2d.fillOval(x, y - circleSize/2, circleSize, circleSize);
                    g2d.setColor(Color.BLACK);
                    g2d.drawString("1,0 m/s", x + circleSize + 8, y + 4);
                    
                    // Item 3 - 1,5 m/s (Morado oscuro)
                    y += spacing;
                    g2d.setColor(new Color(48, 0, 138));
                    g2d.fillOval(x, y - circleSize/2, circleSize, circleSize);
                    g2d.setColor(Color.BLACK);
                    g2d.drawString("1,5 m/s", x + circleSize + 8, y + 4);
                }
            };
            
            // Configurar el panel de leyenda
            legendPanel.setOpaque(true);
            legendPanel.setBackground(new Color(255, 255, 255, 240));
            legendPanel.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createLineBorder(new Color(100, 100, 100), 2),
                BorderFactory.createEmptyBorder(5, 5, 5, 5)
            ));
            
            // Tamaño y posición
            final int width = 120;
            final int height = 90;
            final int margin = 15;
            
            legendPanel.setBounds(
                displayPanel.getWidth() - width - margin,
                margin,
                width,
                height
            );
            
            // Listener para reposicionar si cambia el tamaño
            displayPanel.addComponentListener(new ComponentAdapter() {
                @Override
                public void componentResized(ComponentEvent evt) {
                    legendPanel.setBounds(
                        displayPanel.getWidth() - width - margin,
                        margin,
                        width,
                        height
                    );
                }
            });
            
            displayPanel.setLayout(null);
            displayPanel.add(legendPanel);
            displayPanel.setComponentZOrder(legendPanel, 0);
            displayPanel.revalidate();
            displayPanel.repaint();
            
        } catch (Exception ex) {
            System.out.println("Error al agregar leyenda ");
            ex.printStackTrace();
        }
    }
}