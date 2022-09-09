#include <cstdio>
#include <cstdlib>
#include <glpk.h>
#include <string>

#include "2d_strip_packing_lp.hpp"

struct bin_t {
  int w_;
  int h_;
  int x_;
  int y_;
}; // struct bin //

namespace mv {

template<>
struct rectangle_traits<bin_t> {
  typedef bin_t rect_t;
  typedef int unit_t;
  static unit_t width(const rect_t& r) { return r.w_; }
  static unit_t height(const rect_t& r) { return r.h_; }
}; // struct rectangle_traits //

} // namespace mv //

typedef mv::rectangle_traits<bin_t> traits;
typedef mv::Strip_Packing_Linear_Program<traits> strip_packing_lp_t;
typedef typename strip_packing_lp_t::constraint_matrix_t constraint_matrix_t;
typedef typename strip_packing_lp_t::matrix_entry_t matrix_entry_t;
typedef typename strip_packing_lp_t::variable_iterator_t variable_iterator_t;

////////////////////////////////////////////////////////////////////////////////
// Packing_Visualizer: generates a LaTeX code which can be compiled to generate
// visualization of packing.
class Packing_Visualizer {

  public:

    Packing_Visualizer(const std::string& file_name="packing.tex")
      : file_name_(file_name) {}

    template<typename PlacedBinIterator>
    void generate(size_t width, size_t height,
        PlacedBinIterator bin_begin, PlacedBinIterator bin_end) {

      FILE *fptr = fopen(file_name_.c_str(), "w");
      assert(fptr);

      /// header //
      fprintf(fptr,
          "\\documentclass[a4paper,12pt]{article}\n"
          "\\usepackage[margin=0.25in]{geometry}\n"
          "\\usepackage{tikz,pgfplots}\n"
          "\\usetikzlibrary{math}\n");


      fprintf(fptr, "\\begin{document}\n");
      fprintf(fptr, "\\tikzmath{\\binwidth=%lu; \\binheight=%lu;\n"
          "\\inputx=\\binwidth+10; \\inputy=\\binheight; "
          "\\packlabel=\\binwidth/2; }", width, height);

      fprintf(fptr, "\\begin{tikzpicture}[x=0.6cm, y=0.6cm]\n");
      fprintf(fptr,
          "\\draw[thin, step=1.0] (0,0) grid (\\binwidth, \\binheight);\n");

      char const * color_picker[7UL] = { "red", "green",  "yellow", "orange",
          "green", "purple", "magenta" };
      size_t color_mod = 7UL, cid=0UL;

      // draw bins //
      size_t bid=1UL;
      for (PlacedBinIterator bin_itr=bin_begin; bin_itr!=bin_end; ++bin_itr) {
        bin_t curr_bin = *bin_itr;
        fprintf(fptr, "\\draw [fill=%s] (%d,%d) rectangle (%d,%d) "
            "node[midway]{\\bf $B_{%lu}$ };\n", color_picker[(bid++)%color_mod],
            curr_bin.x_, curr_bin.y_, (curr_bin.x_ + curr_bin.w_),
            (curr_bin.y_ + curr_bin.h_), bid);
      }

      for (PlacedBinIterator bin_itr=bin_begin; bin_itr!=bin_end; ++bin_itr) {
        bin_t curr_bin = *bin_itr;
        fprintf(fptr, "\\draw [fill=black] (%d,%d) circle (0.1cm);\n",
            curr_bin.x_, curr_bin.y_);
      }

      fprintf(fptr, "\\node[text] at (\\packlabel, -1){ Packing };\n");
      fprintf(fptr, "\\node[text] at (\\inputx, \\inputy) {Input Bins};\n");

      size_t upper_bound_y = 0;
      size_t margin_x = width + 10;
      size_t row_spacing = 5;
      size_t base_x = margin_x;
      size_t base_y = upper_bound_y + row_spacing, max_x = 30;


      bid=1UL;
      for (PlacedBinIterator bin_itr=bin_begin; bin_itr!=bin_end; ++bin_itr) {
        bin_t curr_bin = *bin_itr;

        if ((base_x + curr_bin.w_) > max_x) {
          base_x = margin_x;
          base_y = upper_bound_y + row_spacing;
        }

        // update upper_bound_y //
        size_t xmin = base_x, ymin= base_y;
        size_t xmax = xmin+curr_bin.w_, ymax=ymin+curr_bin.h_;

        fprintf(fptr, "\\draw [fill=%s] (%d,%d) rectangle (%d,%d) "
            "node[midway]{\\bf $B_{%lu}$ };\n", color_picker[(bid++)%color_mod],
            curr_bin.x_, curr_bin.y_, (curr_bin.x_ + curr_bin.w_),
            (curr_bin.y_ + curr_bin.h_), bid);
        upper_bound_y = std::max(upper_bound_y, ymax);
      }


      fprintf(fptr, "\\end{tikzpicture}\n");
      fprintf(fptr, "\\end{document}\n");
      fclose(fptr);
    }

  private:

    std::string file_name_;
}; // class Packing_Visualizer //
////////////////////////////////////////////////////////////////////////////////





void ILPSolve_GLPK(const strip_packing_lp_t& packing_lp,
      bool use_relaxed=false) {
  glp_prob *lp_prob;
  const constraint_matrix_t& matrix = packing_lp.get_constraint_matrix();
  size_t row_count = matrix.row_dim_ , col_count = matrix.col_dim_;

  // STEP-1: create problem-instance //
  lp_prob = glp_create_prob();
  glp_set_prob_name(lp_prob, "strip-packer");
  glp_set_obj_dir(lp_prob, GLP_MIN);

  glp_add_rows(lp_prob, row_count);


  int width = packing_lp.get_width();
  int height = packing_lp.get_height();
  int curr_row = row_count + 1UL;
  char name_buf[4096];
  size_t m = matrix.entries_.size();
  int *row_index_array = new int[m+1];
  int *col_index_array = new int[m+1];
  double *value_array = new double[m+1];

  // STEP-2: create rows //
  int eid=1;
  for (const matrix_entry_t& entry : matrix.entries_) {
    if ((entry.index_.row_idx_+1UL) != curr_row) {
      curr_row = entry.index_.row_idx_ + 1UL;
      sprintf(name_buf, "ROW-%d", curr_row);
      glp_set_row_name(lp_prob, curr_row, name_buf);
    }
    int curr_col = entry.index_.col_idx_;
    int curr_value = entry.value_;

    if (curr_col == col_count+1) {
      // set row bounds //
      glp_set_row_bnds(lp_prob, curr_row, GLP_UP, 0.0, double(curr_value));
    } else {
      row_index_array[eid] = curr_row;
      col_index_array[eid] = curr_col;
      value_array[eid] = double(curr_value);
      ++eid;
    }
  }


  // STEP-3: create columns //
  int col_id=1;
  {
    glp_add_cols(lp_prob, col_count);
    for (variable_iterator_t itr=packing_lp.variable_iterator_begin();
          itr!=packing_lp.variable_iterator_end(); ++itr) {
      const char *col_name = *itr;
      glp_set_col_name(lp_prob, col_id, col_name);
      if (col_id != col_count) {
        glp_set_obj_coef(lp_prob, col_id, 0);
      } else {
        glp_set_obj_coef(lp_prob, col_id, double(1));
      }

      if (col_name[0] == 'x') {
        glp_set_col_bnds(lp_prob, col_id, GLP_DB, 0.0, double(width));
      } else if (col_name[0] == 'y') {
        glp_set_col_bnds(lp_prob, col_id, GLP_DB, 0.0, double(height));
      } else if (col_name[0] == 't') {
        glp_set_col_bnds(lp_prob, col_id, GLP_DB, 0.0, double(1.0));
      } else {
        glp_set_col_bnds(lp_prob, col_id, GLP_DB, 0.0, double(height));
      }
      glp_set_col_kind(lp_prob, col_id, GLP_IV);
      ++col_id;
    }
  }

  glp_load_matrix(lp_prob, eid-1, row_index_array, col_index_array,
        value_array);

  glp_iocp param;
  glp_write_lp(lp_prob, NULL, "packing_lp.cplex");


  printf("\n *** Solution ****\n");
  if (use_relaxed) {
    glp_simplex(lp_prob, NULL);
    printf("Obj = %g \n", glp_get_obj_val(lp_prob));
  } else {
    glp_init_iocp(&param);
    param.presolve= GLP_ON;
    int err = glp_intopt(lp_prob, &param);
    if (err) {
      fprintf(stderr, "GLP ILP Solver: failed\n");
    }
    printf("Obj = %g \n", glp_mip_obj_val(lp_prob));
  }

  col_id=1;
  for (variable_iterator_t itr=packing_lp.variable_iterator_begin();
        itr!=packing_lp.variable_iterator_end(); ++itr) {
    if (use_relaxed) {
      printf("%s = %f\n", *itr, glp_get_col_prim(lp_prob, col_id));
    } else {
      printf("%s = %f\n", *itr, glp_mip_col_val(lp_prob, col_id));
    }
    ++col_id;
  }
  printf("**************\n");


  if (row_index_array) { 
    delete [] row_index_array;
    row_index_array = NULL;
  }

  if (col_index_array) { 
    delete [] col_index_array;
    col_index_array = NULL;
  }

  if (value_array) {
    delete [] value_array;
    value_array = NULL;
  }


}


int main(int argc, char** argv) {
  strip_packing_lp_t strip_packing_lp;
  size_t n = 0;

  std::vector<bin_t> input_bins;
  printf("Enter number of bins:\n");
  if (scanf("%lu", &n) != 1UL) {
    fprintf(stderr, "Parse Error:\n");
    return -1;
  }
  printf("Enter width and height:\n");
  int w, h_bound;
  if (scanf("%d%d", &w, &h_bound) != 2UL) {
    fprintf(stderr, "Parse Error:\n");
    return -1;
  }

  for (size_t i=0; i<n; i++) {
    bin_t bin;
    if (scanf("%d %d", &(bin.w_), &(bin.h_)) != 2UL) {
      fprintf(stderr, "Parse Error:\n");
      return -1;
    }
    input_bins.push_back(bin);
  }

  strip_packing_lp.reset(input_bins.begin(), input_bins.end(), w, h_bound);
  strip_packing_lp.generate_constraint_matrix();

  const constraint_matrix_t &matrix = strip_packing_lp.get_constraint_matrix();

  printf("=====Constraint Matrix======\n");
  printf(" rows=%lu cols=%lu\n", matrix.row_dim_, matrix.col_dim_);

  size_t curr_row = matrix.row_dim_ + 1UL;

  for (const matrix_entry_t& entry : matrix.entries_) {
    if (entry.index_.row_idx_ != curr_row) {
      curr_row = entry.index_.row_idx_;
      printf("\n");
    }
    printf("a[%lu,%lu]=%d ", entry.index_.row_idx_, entry.index_.col_idx_,
        entry.value_);
  }

  printf("\nVariables:\n");
  for (variable_iterator_t itr=strip_packing_lp.variable_iterator_begin();
        itr!=strip_packing_lp.variable_iterator_end(); ++itr) {
    printf("%s ", *itr);
  }
  printf("\n==========================\n");
  ILPSolve_GLPK(strip_packing_lp);

  return 0;
}
