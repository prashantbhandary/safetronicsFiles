#include "spo2_algorithm.h"

// Simplified SPO2 calculation algorithm
void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, int32_t *pn_heart_rate, int8_t *pch_hr_valid) {
  
  // Initialize output parameters
  *pn_spo2 = -999;
  *pch_spo2_valid = 0;
  *pn_heart_rate = -999;
  *pch_hr_valid = 0;
  
  if (n_ir_buffer_length < 100) {
    return;
  }
  
  // Calculate DC and AC components
  uint32_t ir_mean = 0;
  uint32_t red_mean = 0;
  
  for (int i = 0; i < n_ir_buffer_length; i++) {
    ir_mean += pun_ir_buffer[i];
    red_mean += pun_red_buffer[i];
  }
  
  ir_mean /= n_ir_buffer_length;
  red_mean /= n_ir_buffer_length;
  
  // Calculate AC components (simplified)
  uint32_t ir_ac = 0;
  uint32_t red_ac = 0;
  
  for (int i = 0; i < n_ir_buffer_length; i++) {
    if (pun_ir_buffer[i] > ir_mean) {
      ir_ac += pun_ir_buffer[i] - ir_mean;
    } else {
      ir_ac += ir_mean - pun_ir_buffer[i];
    }
    
    if (pun_red_buffer[i] > red_mean) {
      red_ac += pun_red_buffer[i] - red_mean;
    } else {
      red_ac += red_mean - pun_red_buffer[i];
    }
  }
  
  // Calculate R ratio
  if (ir_mean > 0 && red_mean > 0) {
    float r_ratio = (float)(red_ac * ir_mean) / (float)(ir_ac * red_mean);
    
    // Simple SPO2 calculation using empirical formula
    float spo2_float = 110.0 - 25.0 * r_ratio;
    
    if (spo2_float > 70.0 && spo2_float < 100.0) {
      *pn_spo2 = (int32_t)spo2_float;
      *pch_spo2_valid = 1;
    }
  }
  
  // Simple heart rate detection by counting peaks
  int peak_count = 0;
  uint32_t threshold = ir_mean + (ir_ac / n_ir_buffer_length);
  
  for (int i = 1; i < n_ir_buffer_length - 1; i++) {
    if (pun_ir_buffer[i] > threshold && 
        pun_ir_buffer[i] > pun_ir_buffer[i-1] && 
        pun_ir_buffer[i] > pun_ir_buffer[i+1]) {
      peak_count++;
    }
  }
  
  if (peak_count > 0) {
    // Assuming 4 seconds of data (100 samples at 25Hz)
    int heart_rate = (peak_count * 60) / 4;
    
    if (heart_rate > 50 && heart_rate < 200) {
      *pn_heart_rate = heart_rate;
      *pch_hr_valid = 1;
    }
  }
}