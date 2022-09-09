#include <cstring>
#include <string>
#include <list>
#include <map>
#include <unordered_map>
#include <unordered_set>


#include <gtest/gtest.h>

#include "2d_strip_packing_lp.hpp"
#include "2d_strip_packing_glpk_solver.hpp"

#include "strip_packing_unit_test_utils.hpp"

struct rect_t {
  typedef int unit_t;
  unit_t width_;
  unit_t height_;
  unit_t x_;
  unit_t y_;

  rect_t(unit_t w=0, unit_t h=0)
    : width_(w), height_(h), x_(0), y_(0) {}

  rect_t(unit_t x, unit_t y, unit_t w, unit_t h)
    : width_(w), height_(h), x_(x), y_(y) {}

  void print() const {
    printf("[xmin ymin xmax ymax] = [%d %d %d %d]\n", x_, y_,
        x_+width_, y_+height_);
  }

  void place(unit_t x, unit_t y) {
    x_ = x; y_ = y;
  }

  static unit_t width(const rect_t& r) { return r.width_; }
  static unit_t height(const rect_t& r) { return r.height_; }
  static unit_t llx(const rect_t& r) { return r.x_; }
  static unit_t lly(const rect_t& r) { return r.y_; }

  static void xinterval(const rect_t& r, unit_t& xbeg, unit_t& xend) {
    xbeg = r.x_;
    xend = xbeg + r.width_;
  }

  static void yinterval(const rect_t& r, unit_t& ybeg, unit_t& yend) {
    ybeg = r.y_;
    yend = ybeg + r.height_;
  }

  static void create_rect(const unit_t& xmin, const unit_t& ymin,
      const unit_t& xmax, const unit_t& ymax, rect_t& output) {
    output.x_ = xmin; output.y_ = ymin;
    output.width_ = (xmax - xmin);
    output.height_ = (ymax - ymin);
  }

};  // struct rect_t //
typedef typename rect_t::unit_t unit_t;

namespace mv {

template<>
struct rectangle_traits<rect_t> : public rect_t {
}; // struct rectangle_traits<rect_t> //

} // namespace mv //

typedef mv::strip_packing_unit_tests::Operation_Dag dag_t;
typedef typename dag_t::operation_t operation_t;
typedef typename dag_t::const_operation_iterator_t const_operation_iterator_t;

typedef typename mv::Strip_Packing_Linear_Program<rect_t, dag_t>
  strip_packing_lp_t;
typedef typename strip_packing_lp_t::variable_iterator_t variable_iterator_t;


TEST(Strip_Packing_Linear_Program,  variable_iterator_basic_test){
  variable_iterator_t vitr, vitr_end;
  EXPECT_TRUE(vitr == vitr_end);
}

TEST(Strip_Packing_Linear_Program, variable_iterator_3_rects) {
  strip_packing_lp_t strip_packing_lp;
  rect_t input[3] = { rect_t(5,10), rect_t(6,8), rect_t(4,3) };

  strip_packing_lp.reset(&input[0], &input[3], 20, 20);

  std::vector<std::string> expected_variables = { "x_1", "y_1", "x_2", "y_2",
   "x_3", "y_3", "t_R_1_2", "t_L_1_2", "t_U_1_2", "t_B_1_2", "t_R_1_3",
   "t_L_1_3", "t_U_1_3", "t_B_1_3", "t_R_2_3", "t_L_2_3", "t_U_2_3", "t_B_2_3",
   "g_1", "g_2", "g_3", "z_H"};

  variable_iterator_t vitr = strip_packing_lp.variable_iterator_begin();
  variable_iterator_t vitr_end = strip_packing_lp.variable_iterator_end();
  size_t correct_names = 0UL;
  for (;vitr != vitr_end; ++vitr,++correct_names) {
    ASSERT_TRUE(correct_names < expected_variables.size());
    std::string variable_name(*vitr);
    printf(" %s ", variable_name.c_str());
  }
  printf("\n");

  EXPECT_EQ(correct_names, expected_variables.size());
}


typedef typename strip_packing_lp_t::coinciding_pair_t coinciding_pair_t;

TEST(Strip_Packing_Linear_Program,
      variable_iterator_3_rects_with_coinciding_pair) {
  strip_packing_lp_t strip_packing_lp;

  rect_t input[3] = { rect_t(5,10), rect_t(6,8), rect_t(4,3) };

  strip_packing_lp.reset(&input[0], &input[3], 20, 20);
  { // coinciding pairs //
    coinciding_pair_t cpairs[2] = { {2,3}, {1,3} };
    strip_packing_lp.add_coinciding_pairs_and_update_variables(cpairs,
          cpairs+2);
  }

  std::vector<std::string> expected_variables = { "x_1", "y_1", "x_2", "y_2",
   "x_3", "y_3", "t_R_1_2", "t_L_1_2", "t_U_1_2", "t_B_1_2", "t_R_1_3",
   "t_L_1_3", "t_U_1_3", "t_B_1_3", "t_R_2_3", "t_L_2_3", "t_U_2_3", "t_B_2_3",
   "g_1", "g_2", "g_3",
   "abs_x_2_3", "abs_y_2_3", "abs_2_3", "t_abs_x_2_3", "t_abs_y_2_3",
      "t_coincide_2_3",
   "abs_x_1_3", "abs_y_1_3", "abs_1_3", "t_abs_x_1_3", "t_abs_y_1_3",
      "t_coincide_1_3",
   "z_H" };

  variable_iterator_t vitr = strip_packing_lp.variable_iterator_begin();
  variable_iterator_t vitr_end = strip_packing_lp.variable_iterator_end();
  size_t correct_names = 0UL;
  for (;vitr != vitr_end; ++vitr,++correct_names) {
    ASSERT_TRUE(correct_names < expected_variables.size());
    std::string variable_name(*vitr);
    printf(" %s ", variable_name.c_str());
  }
  printf("\n");

  EXPECT_EQ(correct_names, expected_variables.size());
}


TEST(Strip_Packing_Linear_Program, get_variable_name_from_col_id) {
  strip_packing_lp_t strip_packing_lp;
  rect_t input[3] = { rect_t(5,10), rect_t(6,8), rect_t(4,3) };

  strip_packing_lp.reset(&input[0], &input[3], 20, 20);

  std::vector<std::string> expected_variables = { "x_1", "y_1", "x_2", "y_2",
   "x_3", "y_3", "t_R_1_2", "t_L_1_2", "t_U_1_2", "t_B_1_2", "t_R_1_3",
   "t_L_1_3", "t_U_1_3", "t_B_1_3", "t_R_2_3", "t_L_2_3", "t_U_2_3", "t_B_2_3",
   "g_1", "g_2", "g_3", "z_H"};

  size_t col_id;
  for (col_id=0; col_id < expected_variables.size(); ++col_id) {
    ASSERT_EQ(strip_packing_lp.get_variable_name_from_col_id(col_id+1UL),
        expected_variables[col_id]);
  }
  ASSERT_EQ(strip_packing_lp.get_variable_name_from_col_id(col_id+1UL), "");
}


TEST(Strip_Packing_Linear_Program, row_constraint_iterator_basic_test) {
  typedef typename strip_packing_lp_t::constraint_matrix_t constraint_matrix_t;
  typedef typename strip_packing_lp_t::row_constraint_iterator_t
      row_constraint_iterator_t;
  typedef typename
      row_constraint_iterator_t::variable_name_coefficient_iterator_t
        variable_name_coefficient_iterator_t;
  
  strip_packing_lp_t strip_packing_lp;

  rect_t input[3] = { rect_t(5,10), rect_t(6,8), rect_t(4,3) };

  strip_packing_lp.reset(&input[0], &input[3], 20, 20);
  strip_packing_lp.generate_constraint_matrix();
  
  const constraint_matrix_t &cmatrix = strip_packing_lp.get_constraint_matrix();
  size_t expected_row_dim = cmatrix.row_dim_;
  size_t expected_col_dim = cmatrix.col_dim_;


  size_t rcount = 1UL;
  size_t max_col_size = 0UL;
  size_t min_col_size = std::numeric_limits<size_t>::max();

  for (row_constraint_iterator_t ritr=strip_packing_lp.constraints_begin();
        ritr != strip_packing_lp.constraints_end(); ++ritr) {
    ++rcount;
    ASSERT_TRUE(rcount <= expected_row_dim);

    size_t ccount = 0UL;
    bool has_bounds_column = false;
    for (variable_name_coefficient_iterator_t vitr=ritr.col_begin();
          vitr!=ritr.col_end(); ++vitr) {
      if (strip_packing_lp.get_row_bounds_col_id() != vitr.column_id()) {
        printf("(%d)%s + ", vitr.coefficient_value(), vitr.variable_name());
      } else {
        has_bounds_column = true;
        printf(" 0 <= %d \n", vitr.coefficient_value());
      }
      ++ccount;
      ASSERT_TRUE(ccount <= expected_col_dim);
    }
    ASSERT_TRUE(has_bounds_column);
    max_col_size = std::max(max_col_size, ccount);
    min_col_size = std::min(min_col_size, ccount);
  }
  ASSERT_EQ(rcount, expected_row_dim);
  ASSERT_TRUE(min_col_size <= expected_col_dim);
  ASSERT_TRUE(max_col_size <= expected_col_dim);
}

TEST(Strip_Packing_Linear_Program, row_constraint_iterator_symbolic_test) {
  typedef typename strip_packing_lp_t::constraint_matrix_t constraint_matrix_t;
  typedef typename strip_packing_lp_t::row_constraint_iterator_t
      row_constraint_iterator_t;
  typedef typename
      row_constraint_iterator_t::variable_name_coefficient_iterator_t
        variable_name_coefficient_iterator_t;
  
  strip_packing_lp_t strip_packing_lp;

  rect_t input[3] = { rect_t(5,10), rect_t(6,8), rect_t(4,3) };

  strip_packing_lp.reset(&input[0], &input[3], 20, 20);
  strip_packing_lp.generate_constraint_matrix();
  
  const constraint_matrix_t &cmatrix = strip_packing_lp.get_constraint_matrix();
  size_t expected_row_dim = cmatrix.row_dim_;
  size_t expected_col_dim = cmatrix.col_dim_;
  std::vector<std::string> expected_variables = { "x_1", "y_1", "x_2", "y_2",
   "x_3", "y_3", "t_R_1_2", "t_L_1_2", "t_U_1_2", "t_B_1_2", "t_R_1_3",
   "t_L_1_3", "t_U_1_3", "t_B_1_3", "t_R_2_3", "t_L_2_3", "t_U_2_3", "t_B_2_3",
   "g_1", "g_2", "g_3", "z_H"};

  std::unordered_set<std::string> unique_symbols;


  size_t rcount = 1UL;
  size_t max_col_size = 0UL;
  size_t min_col_size = std::numeric_limits<size_t>::max();

  for (row_constraint_iterator_t ritr=strip_packing_lp.constraints_begin();
        ritr != strip_packing_lp.constraints_end(); ++ritr) {
    ++rcount;
    ASSERT_TRUE(rcount <= expected_row_dim);
    size_t ccount = 0UL;
    for (variable_name_coefficient_iterator_t vitr=ritr.col_begin();
          vitr!=ritr.col_end(); ++vitr) {
      if (strip_packing_lp.get_row_bounds_col_id() != vitr.column_id()) { 
        unique_symbols.insert( std::string(vitr.variable_name()) );
        ASSERT_TRUE(vitr.column_id() <= expected_variables.size() );
        ASSERT_TRUE(vitr.column_id() >= 1UL);
        ASSERT_EQ( expected_variables[ vitr.column_id()-1UL ],
            std::string(vitr.variable_name()));
      }
      ++ccount;
      ASSERT_TRUE(ccount <= expected_col_dim);
    }
  }
  ASSERT_EQ(rcount, expected_row_dim);
  ASSERT_EQ(expected_variables.size(), unique_symbols.size());
}


TEST(Strip_Packing_Linear_Program, check_column_ids) {
  strip_packing_lp_t strip_packing_lp;
  rect_t input[3] = { rect_t(5,10), rect_t(6,8), rect_t(4,3) };

  strip_packing_lp.reset(&input[0], &input[3], 20, 20);

  std::vector<std::string> expected_variables = { "x_1", "y_1", "x_2", "y_2",
   "x_3", "y_3", "t_R_1_2", "t_L_1_2", "t_U_1_2", "t_B_1_2", "t_R_1_3",
   "t_L_1_3", "t_U_1_3", "t_B_1_3", "t_R_2_3", "t_L_2_3", "t_U_2_3", "t_B_2_3",
   "g_1", "g_2", "g_3", "z_H"};
  std::unordered_map<std::string, size_t> expected_col_ids;

  size_t col_id = 1;
  for (std::string var : expected_variables) {
    expected_col_ids[ var ] = col_id++;
  }

  EXPECT_EQ(strip_packing_lp.get_z_H_column_id(), expected_col_ids["z_H"]);
  EXPECT_EQ(strip_packing_lp.get_col_id_LEFT(1,3), expected_col_ids["t_L_1_3"]);
  EXPECT_EQ(strip_packing_lp.get_col_id_RIGHT(1,2), expected_col_ids["t_R_1_2"]);
  EXPECT_EQ(strip_packing_lp.get_row_bounds_col_id(),
        (expected_variables.size() + 1UL) );
}

TEST(Overlap_Util, overlap_tests) {

  rect_t a(1,1, 10, 15), b(20, 30, 100, 200);
  rect_t c(11,16, 10, 15), d(0, 0, 20, 30);

  EXPECT_FALSE(mv::Overlap_Util::overlapping_rectangles(a,b));
  EXPECT_TRUE(mv::Overlap_Util::overlapping_rectangles(a,a));
  EXPECT_TRUE(mv::Overlap_Util::overlapping_rectangles(b,b));

  // corner touch //

  EXPECT_TRUE(mv::Overlap_Util::overlapping_rectangles(a,c)); // top corner //
  EXPECT_TRUE(mv::Overlap_Util::overlapping_rectangles(b,d)); // ll corner //

  rect_t a_inside(2,2, 1,1), b_inside(25,25,2,3);

  EXPECT_TRUE(mv::Overlap_Util::overlapping_rectangles(a, a_inside));
  EXPECT_FALSE(mv::Overlap_Util::overlapping_rectangles(b, b_inside));
  EXPECT_FALSE(mv::Overlap_Util::overlapping_rectangles(a_inside, b_inside));
}




////////////////////////////////////////////////////////////////////////////////
// Packing_Visualizer: generates a LaTeX code which can be compiled to generate
// visualization of packing.
class Packing_Visualizer {

  public:

    Packing_Visualizer(const std::string& file_name="packing_visualizer.tex")
      : file_name_(file_name) {}


    typedef std::map<size_t, std::list<operation_t> > level_map_t;

    void get_level_map(const dag_t& input, level_map_t& level_map) {
      std::list<operation_t> zero_in_degree_nodes;
      std::unordered_map<operation_t, size_t> in_degree_map;

      size_t curr_level=0;
      for (const_operation_iterator_t citr=input.begin(); citr!=input.end();
            ++citr) {
        size_t in_degree = 0UL;
        for (const_operation_iterator_t
            pitr=dag_t::incoming_operations_begin(input,*citr);
            pitr!=dag_t::incoming_operations_end(input, *citr); ++pitr) {
          ++in_degree;
        }
        in_degree_map[*citr] = in_degree;

        if (!in_degree) {
          level_map[curr_level].push_back( *citr );
        }
      }

      
      while (level_map.find(curr_level) != level_map.end()) {

        std::list<operation_t> const &curr_level_nodes = level_map[curr_level];

        for (operation_t op : curr_level_nodes) {
          for (const_operation_iterator_t citr = input.begin(op);
              citr!=input.end(op); ++citr) {
            operation_t child_op = *citr;
            in_degree_map[child_op]--;
            if (!in_degree_map[child_op]) {
              level_map[curr_level+1].push_back(child_op);
            }
          }
        }
        ++curr_level;
      }
    }

    typedef double const * const_double_ptr_t;

    template<typename VariableValueIterator=const_double_ptr_t>
    void generate_from_solution(const dag_t& dag, size_t width, size_t height,
        VariableValueIterator solution_begin,
        VariableValueIterator solution_end) {
      ++solution_begin; // 1 indexing //
      assert( solution_begin != solution_end);

      std::vector<rect_t> placements;
      for (const_operation_iterator_t nitr=dag.begin(); nitr!=dag.end();
            ++nitr) {
        rect_t rect;
        assert(solution_begin != solution_end);
        rect.x_ = unit_t( *solution_begin );
        ++solution_begin;
        assert(solution_begin != solution_end);
        rect.y_ = unit_t( *solution_begin );
        ++solution_begin;

        rect.width_ = dag_t::width(dag, *nitr);
        rect.height_ = dag_t::height(dag, *nitr);
        placements.push_back( rect );
      }
      generate(dag, width, height, placements.begin(), placements.end());
    }

    template<typename PlacedBinIterator>
    void generate(const dag_t& dag, size_t width, size_t height,
        PlacedBinIterator bin_begin, PlacedBinIterator bin_end) {

      level_map_t level_map;
      std::unordered_map<operation_t, rect_t> rect_map;
      std::vector<operation_t> node_array;
      std::unordered_map<operation_t, size_t> node_id_map;

      for (const_operation_iterator_t nitr=dag.begin(); nitr!=dag.end();
            ++nitr) {
        node_id_map[*nitr] = node_array.size();
        node_array.push_back( *nitr );
      }

      get_level_map(dag, level_map);
      size_t idx = 0UL;
      for (PlacedBinIterator bitr=bin_begin; bitr!=bin_end; ++bitr, ++idx) {
        rect_map[ node_array[idx] ] = *bitr;
      }


      FILE *fptr = fopen(file_name_.c_str(), "w");
      assert(fptr);

      /// header //
      fprintf(fptr,
          "\\documentclass[tikz,border=5pt]{standalone}\n"
          "\\usepackage{tikz,pgfplots}\n"
          "\\usetikzlibrary{math}\n");


      fprintf(fptr, "\\begin{document}\n");
      fprintf(fptr, "\\tikzmath{\\binwidth=%lu; \\binheight=%lu;\n"
          "\\binwidthplusone=\\binwidth+1; \\binwidthminusone=\\binwidth-1;\n"
          "\\inputx=\\binwidth+10; \\inputy=\\binheight; "
          "\\packlabel=\\binwidth/2; }", width, height);

      fprintf(fptr, "\\begin{tikzpicture}[x=0.6cm, y=0.6cm]\n");
      fprintf(fptr,
          "\\draw[thin, step=1.0] (0,0) grid (\\binwidth, \\binheight);\n");

      char const * color_picker[7UL] = { "red", "green",  "yellow", "orange",
          "pink", "purple", "brown" };
      size_t color_mod = 7UL, cid=0UL;

      // draw bins //
      size_t bid=0UL;
      int makespan = 0;
      for (PlacedBinIterator bin_itr=bin_begin; bin_itr!=bin_end; ++bin_itr,
            ++bid) {
        rect_t curr_bin = *bin_itr;

        if (curr_bin.width_) {
          fprintf(fptr, "\\draw [fill=%s] (%d,%d) rectangle (%d,%d) "
              "node[midway]{\\bf %s };\n", color_picker[(bid)%color_mod],
              curr_bin.x_, curr_bin.y_, (curr_bin.x_ + curr_bin.width_),
              (curr_bin.y_ + curr_bin.height_),
              dag_t::operation_name( node_array[bid]) );
        } else {
          fprintf(fptr, "\\draw [ultrathick,%s,line width=2mm] "
              "(%d,%d) -- (%d,%d) "
              "node[midway,black]{\\bf %s };\n", color_picker[(bid)%color_mod],
              curr_bin.x_, curr_bin.y_, (curr_bin.x_ + curr_bin.width_),
              (curr_bin.y_ + curr_bin.height_),
              dag_t::operation_name( node_array[bid]) );
        }

        makespan = std::max(makespan, (curr_bin.y_ + curr_bin.height_) );
      }

      fprintf(fptr, "\\draw[<->,ultrathick,dashed,red] (-1,%d) -- (%d,%d) "
          "node[midway,above]{Makespan=%d};\n", makespan,int(width)+1,makespan,
            makespan);

      for (PlacedBinIterator bin_itr=bin_begin; bin_itr!=bin_end; ++bin_itr) {
        rect_t curr_bin = *bin_itr;
        fprintf(fptr, "\\draw [fill=black] (%d,%d) circle (0.1cm);\n",
            curr_bin.x_, curr_bin.y_);
      }

      fprintf(fptr, "\\node[text] at "
          "(\\packlabel, -1){ Strip Packing Solution};\n");
      fprintf(fptr, "\\node[text] at (\\inputx, 0) {Input Bins + DAG "
            "(w=%lu $h_{fsg}=%lu$)};\n", width, height);

      size_t upper_bound_y = 0;
      size_t margin_x = width + 5;
      size_t row_spacing = 2, col_spacing=2;
      size_t base_x = margin_x;
      size_t base_y = upper_bound_y + row_spacing, max_x = 30;

      fprintf(fptr, "\n\n %%Input \n");

      for (auto litr=level_map.begin(); litr!=level_map.end(); ++litr) {

        base_x = margin_x;
        base_y = upper_bound_y + row_spacing;
        for (operation_t lop : litr->second) {
          rect_t curr_bin = rect_map[ lop ];
          bid = node_id_map[ lop ];

          // update upper_bound_y //
          size_t xmin = base_x, ymin= base_y;
          size_t xmax = xmin+curr_bin.width_, ymax=ymin+curr_bin.height_;

          if (xmin != xmax) {
            fprintf(fptr, "\\draw [fill=%s] (%lu,%lu) rectangle (%lu,%lu) "
                "node[midway] (%lu) {\\bf %s };\n",
                color_picker[(bid)%color_mod],
                xmin, ymin, xmax, ymax, bid, dag_t::operation_name(lop) );
          } else {
            fprintf(fptr, "\\draw [ultrathick,%s,line width=2mm] "
                "(%lu,%lu) -- (%lu,%lu) "
                "node[midway,black] (%lu) {\\bf %s };\n",
                color_picker[(bid)%color_mod],
                xmin, ymin, xmax, ymax, bid, dag_t::operation_name(lop) );
          }


          upper_bound_y = std::max(upper_bound_y, ymax);
          base_x = xmax + col_spacing;
        }
      }

      fprintf(fptr, "\n\n %%Edges \n");

      for (const_operation_iterator_t nitr=dag.begin(); nitr!=dag.end();
            ++nitr) {
        for (const_operation_iterator_t citr=dag.begin(*nitr);
              citr!=dag.end(*nitr); ++citr) {
          fprintf(fptr, "\\path[->,ultrathick,line width=0.5mm] "
              "(%lu) edge (%lu);\n", node_id_map[*nitr], node_id_map[*citr]);
        }
      }

      fprintf(fptr, "\\end{tikzpicture}\n");
      fprintf(fptr, "\\end{document}\n");
      fclose(fptr);
    }

    template<typename PlacedBinIterator>
    void generate(size_t width, size_t height,
        PlacedBinIterator bin_begin, PlacedBinIterator bin_end) {

      FILE *fptr = fopen(file_name_.c_str(), "w");
      assert(fptr);

      /// header //
      fprintf(fptr,
          "\\documentclass[tikz,border=5p]{standalone}\n"
          "\\usepackage{tikz,pgfplots}\n"
          "\\usetikzlibrary{math}\n");


      fprintf(fptr, "\\begin{document}\n");
      fprintf(fptr, "\\tikzmath{\\binwidth=%lu; \\binheight=%lu;\n"
          "\\binwidthplusone=\\binwidth+1; \\binwidthminusone=\\binwidth-1;\n"
          "\\inputx=\\binwidth+10; \\inputy=\\binheight; "
          "\\packlabel=\\binwidth/2; }", width, height);

      fprintf(fptr, "\\begin{tikzpicture}[x=0.6cm, y=0.6cm]\n");
      fprintf(fptr,
          "\\draw[thin, step=1.0] (0,0) grid (\\binwidth, \\binheight);\n");

      char const * color_picker[7UL] = { "red", "green",  "yellow", "orange",
          "pink", "purple", "brown" };
      size_t color_mod = 7UL, cid=0UL;

      // draw bins //
      size_t bid=1UL;
      int makespan = 0;
      for (PlacedBinIterator bin_itr=bin_begin; bin_itr!=bin_end; ++bin_itr) {
        rect_t curr_bin = *bin_itr;
        fprintf(fptr, "\\draw [fill=%s] (%d,%d) rectangle (%d,%d) "
            "node[midway]{\\bf $B_{%lu}$ };\n", color_picker[(bid++)%color_mod],
            curr_bin.x_, curr_bin.y_, (curr_bin.x_ + curr_bin.width_),
            (curr_bin.y_ + curr_bin.height_), bid);

        makespan = std::max(makespan, (curr_bin.y_ + curr_bin.height_) );
      }

      fprintf(fptr, "\\draw[<->,ultrathick,dashed,red] (-1,%d) -- (%d,%d) "
          "node[above]{Makespan=%d};\n", makespan,int(width)+1,makespan,
            makespan);

      for (PlacedBinIterator bin_itr=bin_begin; bin_itr!=bin_end; ++bin_itr) {
        rect_t curr_bin = *bin_itr;
        fprintf(fptr, "\\draw [fill=black] (%d,%d) circle (0.1cm);\n",
            curr_bin.x_, curr_bin.y_);
      }

      fprintf(fptr, "\\node[text] at "
          "(\\packlabel, -1){ Strip Packing Solution};\n");
      fprintf(fptr, "\\node[text] at (\\inputx, 0) {Input Bins "
            "(w=%lu $h_{bound}=%lu$)};\n", width, height);

      size_t upper_bound_y = 0;
      size_t margin_x = width + 5;
      size_t row_spacing = 2, col_spacing=2;
      size_t base_x = margin_x;
      size_t base_y = upper_bound_y + row_spacing, max_x = 30;

      fprintf(fptr, "\n\n %%Input \n");

      bid=1UL;
      for (PlacedBinIterator bin_itr=bin_begin; bin_itr!=bin_end; ++bin_itr) {
        rect_t curr_bin = *bin_itr;

        if ((base_x + curr_bin.width_) > max_x) {
          base_x = margin_x;
          base_y = upper_bound_y + row_spacing;
        }

        // update upper_bound_y //
        size_t xmin = base_x, ymin= base_y;
        size_t xmax = xmin+curr_bin.width_, ymax=ymin+curr_bin.height_;

        fprintf(fptr, "\\draw [fill=%s] (%lu,%lu) rectangle (%lu,%lu) "
            "node[midway]{\\bf $B_{%lu}$ };\n", color_picker[(bid++)%color_mod],
            xmin, ymin, xmax, ymax, bid);
        upper_bound_y = std::max(upper_bound_y, ymax);
        base_x = xmax + col_spacing;
      }


      fprintf(fptr, "\\end{tikzpicture}\n");
      fprintf(fptr, "\\end{document}\n");
      fclose(fptr);
    }

  private:

    std::string file_name_;
}; // class Packing_Visualizer //
////////////////////////////////////////////////////////////////////////////////



typedef mv::Strip_Packing_GLPK_Solver<rect_t, dag_t> glpk_solver_t;
typedef typename glpk_solver_t::solution_iterator_t solution_iterator_t;
typedef typename glpk_solver_t::placement_t placement_t;

TEST(Strip_Packing_GLPK_Solver, basic_solver_test) {

  rect_t input[3] = { rect_t(5,10), rect_t(6,8), rect_t(4,2) };
  glpk_solver_t solver;
  size_t bin_width = 11UL;
  size_t bin_height_bound = 10UL;

  solution_iterator_t sol_begin = solver.solution_begin(input, input+3UL,
        bin_width, bin_height_bound, true), sol_end = solver.solution_end(); 
  size_t output_count=0;

  while (sol_begin != sol_end) {
    ASSERT_TRUE(output_count <= 3UL); 

    placement_t placement = *sol_begin;

    input[output_count].place(placement.x_, placement.y_);

    input[output_count].print();

    ++output_count;
    ++sol_begin;
  }
  ASSERT_EQ(output_count, 3UL);


  // Check non-overlapping //
  for (size_t i=0; i<3UL; i++) {
    for (size_t j=i+1UL; j<3UL; j++) {
      EXPECT_FALSE(mv::Overlap_Util::overlapping_rectangles_without_touching(
              input[i], input[j]));
    }
  }

}


TEST(Strip_Packing_GLPK_Solver, packing_visual_5) {
  rect_t input[5] = { rect_t(4,5), rect_t(7,2), rect_t(5,10),
      rect_t(6,8), rect_t(4,2)};
  glpk_solver_t solver;
  size_t bin_width = 11UL;
  size_t bin_height_bound = 20UL;
  size_t N=5UL;

  solution_iterator_t sol_begin = solver.solution_begin(input, input+N,
        bin_width, bin_height_bound), sol_end = solver.solution_end(); 
  size_t output_count=0;

  while (sol_begin != sol_end) {
    ASSERT_TRUE(output_count <= N); 

    placement_t placement = *sol_begin;

    input[output_count].place(placement.x_, placement.y_);

    input[output_count].print();

    ++output_count;
    ++sol_begin;
  }
  ASSERT_EQ(output_count, N);


  // Check non-overlapping //
  for (size_t i=0; i<N; i++) {
    for (size_t j=i+1UL; j<N; j++) {
      EXPECT_FALSE(mv::Overlap_Util::overlapping_rectangles_without_touching(
              input[i], input[j]));
    }
  }

  Packing_Visualizer packing_visualizer;
  packing_visualizer.generate(bin_width, bin_height_bound, input, input+N);
}


TEST(Strip_Packing_GLPK_Solver, 5_rects_with_zero_squish_paircoinciding_pairs) {
  rect_t input[5] = { rect_t(4,5), rect_t(5,8), rect_t(5,10),
      rect_t(6,8), rect_t(4,2)};
  glpk_solver_t solver;
  size_t bin_width = 11UL;
  size_t bin_height_bound = 20UL;
  size_t N=5UL;

  std::vector<coinciding_pair_t> coinciding_pairs;
  solution_iterator_t sol_begin = solver.solution_begin_with_coinciding_pairs(
      input, input+N, coinciding_pairs.begin(), coinciding_pairs.end(),
        bin_width, bin_height_bound), sol_end = solver.solution_end(); 
  size_t output_count=0;

  while (sol_begin != sol_end) {
    ASSERT_TRUE(output_count <= N); 

    placement_t placement = *sol_begin;

    input[output_count].place(placement.x_, placement.y_);

    input[output_count].print();

    ++output_count;
    ++sol_begin;
  }
  ASSERT_EQ(output_count, N);


  // Since we relaxed non-overlap constraint to coinciding constraint
  // we should except R1 and R5 have same values of placement variables.
  // all other pairs should be non-overlapping //

  for (size_t i=0; i<N; i++) {
    for (size_t j=i+1UL; j<N; j++) {
        EXPECT_FALSE(mv::Overlap_Util::overlapping_rectangles_without_touching(
              input[i], input[j]));
    }
  }

  Packing_Visualizer packing_visualizer("no-squish-pairs.tex");
  packing_visualizer.generate(bin_width, bin_height_bound, input, input+N);
}


TEST(Strip_Packing_GLPK_Solver, 5_rects_with_one_squish_paircoinciding_pairs) {
  rect_t input[5] = { rect_t(4,5), rect_t(5,8), rect_t(5,10),
      rect_t(6,8), rect_t(4,2)};
  glpk_solver_t solver;
  size_t bin_width = 11UL;
  size_t bin_height_bound = 20UL;
  size_t N=5UL;

  std::vector<coinciding_pair_t> coinciding_pairs;
  {
    // allow rect-1 and rect-5 coincide //
    coinciding_pairs.push_back(coinciding_pair_t(1UL, 5UL) );
  }
  solution_iterator_t sol_begin = solver.solution_begin_with_coinciding_pairs(
      input, input+N, coinciding_pairs.begin(), coinciding_pairs.end(),
        bin_width, bin_height_bound), sol_end = solver.solution_end(); 
  size_t output_count=0;

  while (sol_begin != sol_end) {
    ASSERT_TRUE(output_count <= N); 

    placement_t placement = *sol_begin;

    input[output_count].place(placement.x_, placement.y_);

    input[output_count].print();

    ++output_count;
    ++sol_begin;
  }
  ASSERT_EQ(output_count, N);


  // Since we relaxed non-overlap constraint to coinciding constraint
  // we should except R1 and R5 have same values of placement variables.
  // all other pairs should be non-overlapping //

  for (size_t i=0; i<N; i++) {
    for (size_t j=i+1UL; j<N; j++) {
      if ( !((i==0) && (j==4)) ) {
        EXPECT_FALSE(mv::Overlap_Util::overlapping_rectangles_without_touching(
              input[i], input[j]));
      } else {
        EXPECT_EQ(input[i].x_, input[j].x_);
        EXPECT_EQ(input[i].y_, input[j].y_);
      }
    }
  }

  Packing_Visualizer packing_visualizer("squish-1_5.tex");
  packing_visualizer.generate(bin_width, bin_height_bound, input, input+N);
}

TEST(Strip_Packing_GLPK_Solver, 5_rects_with_two_squish_paircoinciding_pairs) {
  rect_t input[5] = { rect_t(4,3), rect_t(5,8), rect_t(5,10),
      rect_t(6,8), rect_t(4,2)};
  glpk_solver_t solver;
  size_t bin_width = 11UL;
  size_t bin_height_bound = 20UL;
  size_t N=5UL;

  std::vector<coinciding_pair_t> coinciding_pairs;
  {
    // allow rect-1 and rect-5 coincide //
    coinciding_pairs.push_back(coinciding_pair_t(1UL, 5UL) );
    coinciding_pairs.push_back(coinciding_pair_t(3UL, 4UL) );
  }
  solution_iterator_t sol_begin = solver.solution_begin_with_coinciding_pairs(
      input, input+N, coinciding_pairs.begin(), coinciding_pairs.end(),
        bin_width, bin_height_bound), sol_end = solver.solution_end(); 
  size_t output_count=0;

  while (sol_begin != sol_end) {
    ASSERT_TRUE(output_count <= N); 

    placement_t placement = *sol_begin;

    input[output_count].place(placement.x_, placement.y_);

    input[output_count].print();

    ++output_count;
    ++sol_begin;
  }
  ASSERT_EQ(output_count, N);


  // Since we relaxed non-overlap constraint to coinciding constraint
  // we should except R1 and R5 have same values of placement variables.
  // all other pairs should be non-overlapping //

  for (size_t i=0; i<N; i++) {
    for (size_t j=i+1UL; j<N; j++) {
      if ( !((i==0) && (j==4))  && !((i==2UL) && (j==3UL)) ) {
        EXPECT_FALSE(mv::Overlap_Util::overlapping_rectangles_without_touching(
              input[i], input[j]));
      } else {
        EXPECT_EQ(input[i].x_, input[j].x_);
        EXPECT_EQ(input[i].y_, input[j].y_);
      }
    }
  }

  Packing_Visualizer packing_visualizer("squish-1_5_and_3_4.tex");
  packing_visualizer.generate(bin_width, bin_height_bound, input, input+N);
}

TEST(Strip_Packing_GLPK_Solver, no_solution) {
  rect_t input[5] = { rect_t(4,3), rect_t(7,2), rect_t(5,10),
      rect_t(6,8), rect_t(4,2)};
  glpk_solver_t solver;
  size_t bin_width = 10UL;
  size_t bin_height_bound = 15UL;
  size_t N=5UL;

  solution_iterator_t sol_begin = solver.solution_begin(input, input+N,
        bin_width, bin_height_bound), sol_end = solver.solution_end(); 

  // no feasible solution //
  EXPECT_TRUE(sol_begin == sol_end);
}

TEST(Strip_Packing_Linear_Program, basic_dag_test) {
  dag_t input_dag;

  strip_packing_lp_t strip_packing_lp;

  strip_packing_lp.reset_from_dag(input_dag, 10, 25);

  variable_iterator_t vitr_begin=strip_packing_lp.variable_iterator_begin();
  variable_iterator_t vitr_end=strip_packing_lp.variable_iterator_end();

  EXPECT_EQ(strip_packing_lp.dimension(), 0UL);
  EXPECT_TRUE(vitr_begin == vitr_end);
}

////////////////////////////////////////////////////////////////////////////////
class Strip_Packing_Fixture : public testing::Test {
  protected:
    ////////////////////////////////////////////////////////////////////////////
    typedef strip_packing_unit_testing_t sched_traits_t;
    typedef typename sched_traits_t::dag_t dag_t;
    typedef typename std::vector<sp_scheduler_t::scheduled_op_info_t>
      schedule_order_t;
    typedef typename sched_traits_t::dag_editor_t dag_editor_t;
    typedef typename sp_scheduler_t::scheduled_op_info_t scheduled_op_info_t;
    ////////////////////////////////////////////////////////////////////////////

    void SetUp() override {}
    void TearDown() override {}

    typedef typename dag_t::adjacency_map_t adjacency_map_t;
    typedef dag_t::delay_cost_model_t heights_t;
    typedef dag_t::resource_cost_model_t widths_t;

    void reset_input(adjacency_map_t& edges, heights_t& heights,
          widths_t& widths) {
      input_dag_.reset(edges);
      input_dag_.reset_delay_model(heights);
      input_dag_.reset_resource_model(widths);
    }

    static bool starts_with(std::string const& str, char const *prefix) {
      char const * str_in = str.c_str();
      while ( (*prefix != '\0') && (*prefix == *str_in)) {
        ++prefix; ++str_in;
      }
      return *prefix == '\0';
    }


    ////////////////////////////////////////////////////////////////////////////
    // TEST CASES: 
    // test input reused across multiple tests //
    void load_optimal_prefetch_test_input() {
      typename dag_t::adjacency_map_t edges = { {"A", {"B"}}, {"B", {"C"}},
        {"C", {"D"}}, {"D", {}}, {"$W_A$", {"A"}}, {"$W_B$", {"B"}},
        {"$W_C$", {"C"}}, {"$W_D$", {"D"}} };
      // delays = heights //
      typename dag_t::delay_cost_model_t heights =
        { {"A", 10}, {"B", 20}, {"C", 5}, {"D", 5},
          {"$W_A$", 4}, {"$W_B$", 3}, {"$W_C$", 3}, {"$W_D$", 12}};

      // No-prefetch for C //
      typename dag_t::resource_cost_model_t widths =
        { {"$W_A$", 2}, {"A", 4}, {"$W_B$", 2}, {"B", 2}, 
          {"$W_C$", 3}, {"C", 2}, {"$W_D$", 1}, {"D", 2}  };

      typename dag_t::data_op_set_t data_ops = {"$W_A$", "$W_B", "$W_C",
          "$W_D"};
      // Create DAG with dependencies and delays of each of these tasks //
      reset_input(edges, heights, widths);
      input_dag_.reset_data_op_set(data_ops);
    }
    ////////////////////////////////////////////////////////////////////////////

    // runs the production scheduler on input_dag_ and collects the schedule //
    void get_feasible_schedule(schedule_order_t& schedule,
          size_t resource_bound) {
      sp_scheduler_t scheduler_begin(input_dag_, resource_bound), scheduler_end;
      
      for ( ;scheduler_begin != scheduler_end; ++scheduler_begin) {
        (*scheduler_begin).print();
        schedule.push_back( *scheduler_begin );
      }
      schedule.pop_back(); // last is always the spill //
    }




    bool solve_and_verify_iteratively_improved_solution(size_t resource_bound,
        size_t makespan_bound, const char *orig_solution_file="",
        const char *final_solution_file="") {
      
      std::vector<double> solution_from_scheduler;

      // STEP-1: get the feasible solution from scheduler //
      solution_from_scheduler.push_back(0);
      strip_packing_lp_t::derive_all_solution_variables_from_feasible_scheduler(
          input_dag_, resource_bound,
            std::back_inserter(solution_from_scheduler) );

      EXPECT_FALSE( solution_from_scheduler.empty() );

      if (solution_from_scheduler.empty()) { return false; }

      std::pair<bool, size_t> start_solution_result =
        verify_solution(resource_bound, makespan_bound,
            solution_from_scheduler.data(),
            solution_from_scheduler.data()+solution_from_scheduler.size(),
            orig_solution_file);

      EXPECT_TRUE(start_solution_result.first);

      if (!start_solution_result.first) { return false; }


      // STEP-2: improve the feasible solution with LP Solver //

      std::vector<double> improved_solution(solution_from_scheduler.size(),
            0UL);
      std::pair<bool, size_t> improved_solution_result;

      {
        // iteratively improve the solution //
        glpk_solver_t solver;
        solution_iterator_t sol_begin =
          solver.solve_resource_constraint_scheduling_problem(input_dag_,
              resource_bound, makespan_bound,
              solution_from_scheduler.data(),
              solution_from_scheduler.data()+solution_from_scheduler.size(),
              improved_solution.data(),
              improved_solution.data()+improved_solution.size(), false),
          sol_end = solver.solution_end();
        EXPECT_TRUE(sol_begin != sol_end);

        if (sol_begin == sol_end) { return false; }

        improved_solution_result = verify_solution(resource_bound, makespan_bound,
            improved_solution.data(),
            improved_solution.data()+improved_solution.size(),
            final_solution_file );
      }
      
      return (improved_solution_result.second <= start_solution_result.second);
    }

    // bool: success/failure
    // size_t: makespan_bound
    std::pair<bool,size_t> solve_and_verify(size_t resource_bound,
          size_t makespan_bound, bool generate_visual=false,
          const char *file_name="default.tex", bool ignore_precedence=false,
          bool ignore_producer_consumer=true) {
      auto ret = solve_and_verify_general(resource_bound, makespan_bound,
          generate_visual, file_name,
            ignore_precedence, ignore_producer_consumer);

      return ret;
    }

    template<typename RectIterator>
    void update_makespan_bound(size_t &makespan_bound,
        RectIterator rbegin, RectIterator rend) const {
      for (; rbegin != rend; ++rbegin) {
        const rect_t& r = *rbegin;
        makespan_bound = std::max(makespan_bound, size_t(r.y_ + r.height_) );
      }
    }

    // bool: success/failure
    // size_t: makespan_bound
    std::pair<bool,size_t> verify_solution(size_t resource_bound,
          size_t makespan_bound, double const *solution_begin,
          double const *solution_end,
          const char *file_name="") const {
      bool generate_visual = strlen(file_name);
      return solve_and_verify_general(resource_bound, makespan_bound,
          generate_visual, file_name, false, false,
            solution_begin, solution_end);
    }


    // bool: success/failure
    // size_t: makespan_bound
    std::pair<bool,size_t> solve_and_verify_general(size_t resource_bound,
          size_t makespan_bound, bool generate_visual=false,
          const char *file_name="default.tex", bool ignore_precedence=false,
          bool ignore_producer_consumer=true,
          double const *input_solution_begin=NULL,
          double const *input_solution_end=NULL) const {

      glpk_solver_t solver;
      std::vector<operation_t> node_list;
      std::vector<rect_t> placements;
      std::unordered_map<operation_t, size_t> start_time_map;
      std::unordered_map<operation_t, size_t> node_id_map;

      // first collect operations and placements //
      for (const_operation_iterator_t nitr=input_dag_.begin();
            nitr != input_dag_.end(); ++nitr) {
        placements.push_back( rect_t(dag_t::width(input_dag_, *nitr),
                    dag_t::height(input_dag_, *nitr) ) );
        node_id_map[*nitr] = node_list.size();
        node_list.push_back(*nitr);
      }


      bool is_solution_available = (input_solution_begin != input_solution_end);
      size_t make_span = 0;

      if (!is_solution_available) {
        // build placements by solving the LP //
        solution_iterator_t solution_begin =
            solver.solve_resource_constraint_scheduling_problem(input_dag_,
                resource_bound, makespan_bound, ignore_precedence,
                  ignore_producer_consumer);
        solution_iterator_t solution_end = solver.solution_end();

        size_t idx = 0UL;
        while (solution_begin != solution_end) {
          placement_t placement = *solution_begin;
          placements[idx].place(placement.x_, placement.y_);
          start_time_map[ node_list[idx] ]= placement.y_;

          make_span = std::max( make_span,
                size_t(placement.y_ + placements[idx].height_) );
          ++idx;
          ++solution_begin;
        }
      } else {
        placements.clear();
        bool is_valid = collect_placements_from_solution(input_solution_begin,
            input_solution_end, std::back_inserter(placements));
        if (!is_valid) { return std::make_pair(false, make_span); }

        // also update the start_time_map //
        for (size_t i=0; i<placements.size(); i++) {
          start_time_map.insert(std::make_pair(node_list[i], placements[i].y_));
        }

        // update make_span //
        update_makespan_bound(make_span, placements.begin(), placements.end());
      }


      if (generate_visual) {
        Packing_Visualizer packing_visualizer(file_name);
          packing_visualizer.generate(input_dag_,
              resource_bound, makespan_bound, placements.begin(),
                placements.end());
      }

      { 

        // STEP-0: verify all nodes have start time //
        for (const_operation_iterator_t nitr=dag_t::operations_begin(input_dag_);
              nitr!=dag_t::operations_end(input_dag_); ++nitr) {
          operation_t curr_op = *nitr;
          if (start_time_map.find(curr_op) == start_time_map.end()) {
            fprintf(stderr, "Missing start time for %s\n",
                  dag_t::operation_name(curr_op) );
            return std::make_pair(false, make_span);
          }
        } 

        // STEP-1: verify precedence constraints //
        if (!ignore_precedence) {
        for (const_operation_iterator_t
              citr=dag_t::operations_begin(input_dag_);
              citr!=dag_t::operations_end(input_dag_); ++citr) {

          size_t child_start_time = start_time_map[*citr];
          for (const_operation_iterator_t
              pitr=dag_t::incoming_operations_begin(input_dag_,*citr);
              pitr!=dag_t::incoming_operations_end(input_dag_, *citr); ++pitr) {
            size_t parent_start_time = start_time_map[*pitr];
            size_t parent_delta = dag_t::height(input_dag_, *pitr);
            if ( (parent_start_time + parent_delta) > child_start_time) {
              printf("child_start_time=%lu parent_start_time=%lu pdelta=%lu\n",
                  child_start_time, parent_start_time, parent_delta);
              fprintf(stderr, "[Precedence Constraint Violation] %s->%s\n",
                  dag_t::operation_name(*citr), dag_t::operation_name(*pitr) );
              return std::make_pair(false, make_span);
            }

          }
        }
        }

        // STEP-2: Verify non-overlapping //
        for (size_t i=0; i<placements.size(); i++) {
          for (size_t j=i+1; j<placements.size(); j++) {
            if (mv::Overlap_Util::overlapping_rectangles_without_touching(
                  placements[i], placements[j]) ) {
              fprintf(stderr, "[Overlap Constraint Violation]: %s , %s",
                  dag_t::operation_name(node_list[i]),
                  dag_t::operation_name(node_list[j]) );
              return std::make_pair(false, 0UL);
            }
          }
        }


        // STEP-3: verify producer-consumer constraints //
        // foreach node u with non-zero width:
        //
        // let h_delta = max{ h(w) + y(w) | (u,w) \in E }
        // 
        // The rectangle [x(u),y(u), x(u)+w(u), y(u)+h(u)+h_delta] must
        // be non-overlapping with any other rectangle.

        if (!ignore_producer_consumer) {
          // resized placements //
          std::vector<rect_t> resized_placements;

          for (size_t i=0; i<placements.size(); i++) {
            assert(i < node_list.size());
            operation_t op = node_list[i];

            rect_t net_rect = placements[i];
            typename rect_t::unit_t h_orig = (net_rect.y_ + net_rect.height_);
            typename rect_t::unit_t h_max = h_orig; 
            for (const_operation_iterator_t 
                  citr=dag_t::outgoing_operations_begin(input_dag_, op);
                  citr!=dag_t::outgoing_operations_end(input_dag_, op); ++citr)
            {
              operation_t cop = *citr;
              assert(node_id_map.find(cop) != node_id_map.end());
              size_t cop_id = node_id_map[cop];
              assert(cop_id < placements.size());

              const rect_t& cop_rect = placements[cop_id];

              h_max = std::max((cop_rect.height_ + cop_rect.y_) , h_max);
            }

            assert(h_max >= h_orig);
            net_rect.height_ += (h_max - h_orig);
            resized_placements.push_back(net_rect);
          }

          size_t verified_count = 0UL;
          for (size_t i=0; i<resized_placements.size(); i++) {
            for (size_t j=i+1; j<resized_placements.size(); j++) {
              if (mv::Overlap_Util::overlapping_rectangles_without_touching(
                    resized_placements[i], resized_placements[j]) ) {
                fprintf(stderr, "[Producer Consumer Violation]: %s , %s",
                    dag_t::operation_name(node_list[i]),
                    dag_t::operation_name(node_list[j]) );
                return std::make_pair(false, 0UL);
              }
              verified_count++;
            }
          }
          fprintf(stderr, "[Producer Consumer Constraint] verfied=%lu\n",
                verified_count);
        }
      }
      return std::make_pair(true, make_span);
    }


    template<typename VariableValueIterator, typename OutputIterator>
    bool collect_placements_from_solution(VariableValueIterator solution_begin,
        VariableValueIterator solution_end, OutputIterator output) const {

      if (solution_begin == solution_end) { return false; }
      ++solution_begin; // 1 indexing //
      if (solution_begin == solution_end) { return false; }

      rect_t rect;
      const dag_t& dag = input_dag_;
      for (const_operation_iterator_t nitr=dag.begin();
            nitr!=dag.end(); ++nitr) {

        if (solution_begin == solution_end) { return false; }
        rect.x_ = unit_t( *solution_begin );
        ++solution_begin;

        if (solution_begin == solution_end) { return false; }
        rect.y_ = unit_t( *solution_begin );
        ++solution_begin;

        rect.width_ = dag_t::width(dag, *nitr);
        rect.height_ = dag_t::height(dag, *nitr);

        *output = rect;
        ++output;
      }
      return true;
    }


    dag_t input_dag_;
}; // class Strip_Packing_Fixture //
////////////////////////////////////////////////////////////////////////////////



TEST_F(Strip_Packing_Fixture, linear_dag_test) {
  typename dag_t::adjacency_map_t edges =
    { {"A", {"B"}}, {"B", {"C"}}, {"C", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}};
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 6}, {"C", 5}};

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 10;
  size_t makespan_bound = 40;
  std::pair<bool, size_t> result = solve_and_verify(resource_bound,
      makespan_bound, true, "linear_dag.tex");

  EXPECT_TRUE(result.first);
  EXPECT_EQ(result.second, 35);
}

TEST_F(Strip_Packing_Fixture, linear_dag_test_no_precedence) {
  typename dag_t::adjacency_map_t edges =
    { {"A", {"B"}}, {"B", {"C"}}, {"C", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}};
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 6}, {"C", 5}};

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 10;
  size_t makespan_bound = 40;
  std::pair<bool, size_t> result = solve_and_verify(resource_bound,
      makespan_bound, true, "linear_dag_no_precedence.tex", true);

  EXPECT_TRUE(result.first);
  EXPECT_EQ(result.second, 25);
}

TEST_F(Strip_Packing_Fixture, diamond_test) {
  typename dag_t::adjacency_map_t edges =
    { {"A", {"B", "C"}}, {"B", {"D"}}, {"C", {"D"}}, {"D", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}, {"D", 5} };
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 6}, {"C", 2}, {"D", 2} };

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 10;
  size_t makespan_bound = 40;
  std::pair<bool, size_t> result = solve_and_verify(resource_bound,
      makespan_bound, true, "diamond_test.tex");

  EXPECT_TRUE(result.first);
  EXPECT_EQ(result.second, 35);
}

TEST_F(Strip_Packing_Fixture, diamond_test_with_choice) {
  typename dag_t::adjacency_map_t edges =
    { {"A", {"B", "C", "D"}}, {"B", {"E"}}, {"C", {"E"}}, {"D", {"E"}},
      {"E", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}, {"D", 5} , {"E", 5} };
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 6}, {"C", 2}, {"D", 4}, {"E", 8} };

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 10;
  size_t makespan_bound = 45;
  std::pair<bool, size_t> result = solve_and_verify(resource_bound,
      makespan_bound, true, "choice_diamond_test.tex");

  EXPECT_TRUE(result.first);
  EXPECT_EQ(result.second, 35);
}

TEST_F(Strip_Packing_Fixture, linear_dag_test_all_constraints) {
  typename dag_t::adjacency_map_t edges =
    { {"A", {"B"}}, {"B", {"C"}}, {"C", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}};
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 6}, {"C", 4}};

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 10;
  size_t makespan_bound = 40;
  std::pair<bool, size_t> result = solve_and_verify(resource_bound,
      makespan_bound, true, "linear_dag_all_constraints.tex", false, false);

  EXPECT_TRUE(result.first);
  EXPECT_EQ(result.second, 35);
}

TEST_F(Strip_Packing_Fixture, all_constraints_basic) {
  typename dag_t::adjacency_map_t edges =
    { {"A", {"B"}}, {"B", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20} };
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 6} };

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 10;
  size_t makespan_bound = 40;
  std::pair<bool, size_t> result = solve_and_verify(resource_bound,
      makespan_bound, true, "basic_all_constraints.tex", false, false);

  EXPECT_TRUE(result.first);
  EXPECT_EQ(result.second, 30);
}

TEST_F(Strip_Packing_Fixture, diamond_test_all_constraints_no_solution) {
  typename dag_t::adjacency_map_t edges =
    { {"A", {"B", "C"}}, {"B", {"D"}}, {"C", {"D"}}, {"D", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}, {"D", 5} };
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 6}, {"C", 2}, {"D", 2} };

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 10;
  size_t makespan_bound = 40;
  std::pair<bool, size_t> result = solve_and_verify(resource_bound,
      makespan_bound, true, "none.tex", false, false);

  EXPECT_FALSE(result.first);
}

TEST_F(Strip_Packing_Fixture, diamond_test_all_constraints) {
  typename dag_t::adjacency_map_t edges =
    { {"A", {"B", "C"}}, {"B", {"D"}}, {"C", {"D"}}, {"D", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}, {"D", 5} };
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 4}, {"C", 2}, {"D", 2} };

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 10;
  size_t makespan_bound = 40;
  std::pair<bool, size_t> result = solve_and_verify(resource_bound,
      makespan_bound, true, "diamond_test_all_constraints.tex", false, false);

  EXPECT_TRUE(result.first);
  EXPECT_EQ(result.second, 35);
}

TEST_F(Strip_Packing_Fixture, diamond_test_only_precedence) {
  typename dag_t::adjacency_map_t edges =
    { {"A", {"B", "C"}}, {"B", {"D"}}, {"C", {"D"}}, {"D", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}, {"D", 5} };
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 4}, {"C", 2}, {"D", 2} };

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 10;
  size_t makespan_bound = 40;
  std::pair<bool, size_t> result = solve_and_verify(resource_bound,
      makespan_bound, true, "diamond_test_only_precedence.tex", false, true);

  EXPECT_TRUE(result.first);
  EXPECT_EQ(result.second, 35);
}


TEST_F(Strip_Packing_Fixture, diamond_test_only_packing) {
  typename dag_t::adjacency_map_t edges =
    { {"A", {"B", "C"}}, {"B", {"D"}}, {"C", {"D"}}, {"D", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}, {"D", 5} };
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 4}, {"C", 2}, {"D", 2} };

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 10;
  size_t makespan_bound = 40;
  std::pair<bool, size_t> result = solve_and_verify(resource_bound,
      makespan_bound, true, "diamond_test_only_packing.tex", true, true);

  EXPECT_TRUE(result.first);
  EXPECT_EQ(result.second, 20);
}



TEST_F(Strip_Packing_Fixture, linear_dag_zero_resource_task_test) {
  typename dag_t::adjacency_map_t edges =
    { {"A", {"B"}}, {"B", {"C"}}, {"C", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}};
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 0}, {"C", 5}};

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 10;
  size_t makespan_bound = 40;
  std::pair<bool, size_t> result = solve_and_verify(resource_bound,
      makespan_bound, true, "linear_dag_zero_resource.tex");

  EXPECT_TRUE(result.first);
  EXPECT_EQ(result.second, 35);
}

TEST_F(Strip_Packing_Fixture, lock_two_levels_and_reuse_space) {
  typename dag_t::adjacency_map_t edges =
    { {"A", {"B", "C", "E"}}, {"B", {"D"}}, {"C", {"D"}}, {"D", {"E"}},
      {"E", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}, {"D", 5}, {"E", 6} };
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 2}, {"C", 2}, {"D", 2} , {"E", 4} };

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 10;
  size_t makespan_bound = 45;
  std::pair<bool, size_t> result = solve_and_verify(resource_bound,
      makespan_bound, true, "lock_two_levels.tex", false, false);

  EXPECT_TRUE(result.first);
  EXPECT_EQ(result.second, 41);
}

TEST_F(Strip_Packing_Fixture, optimal_weight_prefetch_test_out_of_order_dma) {
  typename dag_t::adjacency_map_t edges = { {"A", {"B"}}, {"B", {"C"}},
    {"C", {"D"}}, {"D", {}}, {"$W_A$", {"A"}}, {"$W_B$", {"B"}},
    {"$W_C$", {"C"}}, {"$W_D$", {"D"}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}, {"D", 5},
      {"$W_A$", 4}, {"$W_B$", 3}, {"$W_C$", 3}, {"$W_D$", 12}};

  // No-prefetch for C //
  typename dag_t::resource_cost_model_t widths =
    { {"$W_A$", 2}, {"A", 4}, {"$W_B$", 2}, {"B", 2}, 
      {"$W_C$", 3}, {"C", 2}, {"$W_D$", 1}, {"D", 2}  };


  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 10;
  size_t makespan_bound = 55;
  std::pair<bool, size_t> result = solve_and_verify(resource_bound,
      makespan_bound, true, "optimal_prefetch_test.tex", false, false);

  EXPECT_TRUE(result.first);
  EXPECT_EQ(result.second, 47);
}

TEST_F(Strip_Packing_Fixture, optimal_weight_prefetch_test_forced_dma_order) {
  typename dag_t::adjacency_map_t edges = { {"A", {"B"}}, {"B", {"C"}},
    {"C", {"D"}}, {"D", {}}, {"$W_A$", {"A", "$W_B$"}},
    {"$W_B$", {"B", "$W_C$"}}, {"$W_C$", {"C", "$W_D$"}}, {"$W_D$", {"D"}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}, {"D", 5},
      {"$W_A$", 4}, {"$W_B$", 3}, {"$W_C$", 3}, {"$W_D$", 12}};

  // No-prefetch for C //
  typename dag_t::resource_cost_model_t widths =
    { {"$W_A$", 2}, {"A", 4}, {"$W_B$", 2}, {"B", 2}, 
      {"$W_C$", 3}, {"C", 2}, {"$W_D$", 1}, {"D", 2}  };


  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 10;
  size_t makespan_bound = 55;
  std::pair<bool, size_t> result = solve_and_verify(resource_bound,
      makespan_bound, true, "forced_dma_optimal_prefetch_test.tex",
        false, false);

  EXPECT_TRUE(result.first);
  EXPECT_EQ(result.second, 54);
}


TEST_F(Strip_Packing_Fixture, simple_prefetch_test) {
  typename dag_t::adjacency_map_t edges = { {"A", {"B"}}, {"B", {}},
    {"$W_A$", {"A"}}, {"$W_B$", {"B"}}  };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"$W_A$", 4}, {"$W_B$", 3} };  

  // No-prefetch for C //
  typename dag_t::resource_cost_model_t widths =
    { {"$W_A$", 2}, {"A", 4}, {"$W_B$", 2}, {"B", 2} };


  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 10;
  size_t makespan_bound = 45;
  std::pair<bool, size_t> result = solve_and_verify(resource_bound,
      makespan_bound, true, "simple_prefetch_test.tex", false, false);

  EXPECT_TRUE(result.first);
  EXPECT_EQ(result.second, 34);
}


TEST_F(Strip_Packing_Fixture, test_packing_with_trival_init_solution) {
  rect_t input[8] = {
    rect_t(4,10), rect_t(2,20), rect_t(2,5), rect_t(2,5), rect_t(2,4),
    rect_t(2, 3), rect_t(3, 3), rect_t(1,12)
  };

  size_t resource_bound = 8;
  size_t makespan_bound = 65;
  size_t opt_make_span = 0UL;
  size_t N=8UL;

  double *init_solution = NULL;
  { 
    // create a in initial solution //
    strip_packing_lp_t strip_packing_lp;
    strip_packing_lp.reset(input, input+N, resource_bound, makespan_bound);
    size_t var_count = strip_packing_lp.total_variable_count();
    init_solution = new double[var_count+1UL]; // one indexing //
    ASSERT_TRUE(init_solution != NULL);

    double running_counter = 0UL;
    size_t ridx = 0UL, sol_idx=1UL;

    for (variable_iterator_t vitr=strip_packing_lp.variable_iterator_begin();
          vitr != strip_packing_lp.variable_iterator_end(); ++vitr) {
      std::string variable_name(*vitr);
      if (starts_with(variable_name, "y_")) {
        init_solution[sol_idx++] = running_counter;

        input[ridx].place(int(0), int(running_counter)); // init solution //


        running_counter += input[ridx].height_;

        ASSERT_TRUE(ridx < N);
        ++ridx;
      } else if (starts_with(variable_name, "t_U") ) {
        init_solution[sol_idx++] = double(1.0);
      } else if (starts_with(variable_name, "z_H")) {
        init_solution[sol_idx++] = double(running_counter);
      } else {
        init_solution[sol_idx++] = double(0.0);
      }
    }

    {
      Packing_Visualizer packing_visualizer("trival_init_solution.tex");
      packing_visualizer.generate(resource_bound, makespan_bound, input,
            input+N);
    }
    
    glpk_solver_t solver;
    solution_iterator_t sol_begin = solver.solution_begin(input, input+N,
          resource_bound, makespan_bound , false, init_solution,
            init_solution+var_count+1UL), sol_end = solver.solution_end(); 
    size_t output_count=0;

    while (sol_begin != sol_end) {
      ASSERT_TRUE(output_count <= N); 

      placement_t placement = *sol_begin;

      input[output_count].place(placement.x_, placement.y_);


      ++output_count;
      ++sol_begin;
    }
    ASSERT_EQ(output_count, N);


    // Check non-overlapping //
    for (size_t i=0; i<N; i++) {
      for (size_t j=i+1UL; j<N; j++) {
        EXPECT_FALSE(mv::Overlap_Util::overlapping_rectangles_without_touching(
                input[i], input[j]));
      }
    }
  } 

  if (init_solution) {
    delete [] init_solution;
  }

  {
    Packing_Visualizer packing_visualizer(
          "final_solution_with_trival_start.tex");
    packing_visualizer.generate(resource_bound, makespan_bound, input, input+N);
  }
}

TEST_F(Strip_Packing_Fixture, test_packing_with_opt_init_solution) {
  //TEST PLAN: solve the problem optimally and pass the optimal solution as
  //starting point //
  rect_t input[8] = {
    rect_t(4,10), rect_t(2,20), rect_t(2,5), rect_t(2,5), rect_t(2,4),
    rect_t(2, 3), rect_t(3, 3), rect_t(1,12)
  };

  size_t resource_bound = 8;
  size_t makespan_bound = 30;
  size_t opt_make_span = 0UL;
  size_t N=8UL;

  double *init_solution = NULL;
  size_t var_count;
  {
    // Create memory for variables //
    {
      strip_packing_lp_t strip_packing_lp;
      strip_packing_lp.reset(input, input+N, resource_bound, makespan_bound);
      var_count = strip_packing_lp.total_variable_count();
      init_solution = new double[var_count+1UL]; // one indexing //
      ASSERT_TRUE(init_solution != NULL);
    }

    // first solve the solution optimally //
    glpk_solver_t solver;
    solution_iterator_t sol_begin = solver.solution_begin(input, input+N,
          resource_bound, makespan_bound, false, NULL, NULL,
          init_solution, init_solution+var_count+1),
                        sol_end = solver.solution_end(); 
    size_t output_count=0;

    while (sol_begin != sol_end) {
      ASSERT_TRUE(output_count <= N); 
      placement_t placement = *sol_begin;
      input[output_count].place(placement.x_, placement.y_);
      ++output_count;
      ++sol_begin;
    }
    ASSERT_EQ(output_count, N);


    // Check non-overlapping //
    for (size_t i=0; i<N; i++) {
      for (size_t j=i+1UL; j<N; j++) {
        EXPECT_FALSE(mv::Overlap_Util::overlapping_rectangles_without_touching(
                input[i], input[j]));
      }
    }
  } 


  { 
    // Run solver with init_solution as starting point //
    glpk_solver_t solver;
    solution_iterator_t sol_begin = solver.solution_begin(input, input+N,
          resource_bound, makespan_bound, false,
          init_solution, init_solution+var_count+1),
                        sol_end = solver.solution_end(); 
    size_t output_count=0;

    while (sol_begin != sol_end) {
      ASSERT_TRUE(output_count <= N); 
      placement_t placement = *sol_begin;
      ASSERT_EQ(input[output_count].x_, placement.x_);
      ASSERT_EQ(input[output_count].y_, placement.y_);
      ++output_count;
      ++sol_begin;
    }
    ASSERT_EQ(output_count, N);
  }

  if (init_solution) {
    delete [] init_solution;
  }


  {
    Packing_Visualizer packing_visualizer("packing_opt_int_solution.tex");
    packing_visualizer.generate(resource_bound, makespan_bound, input, input+N);
  }
}

//TODO(vamsikku): parameterize these tests with parameters shift,fatness //
TEST_F(Strip_Packing_Fixture, test_opt_with_solution_from_narrow_bin_and_shift){
  //TEST PLAN: solve the problem optimally on a narrow bin and use it as
  //a feasible solution to wider bin also shift it by (s_x, s_y) //
  rect_t input[8] = {
    rect_t(4,10), rect_t(2,20), rect_t(2,5), rect_t(2,5), rect_t(2,4),
    rect_t(2, 3), rect_t(3, 3), rect_t(1,12)
  };

  size_t resource_bound = 8;
  size_t makespan_bound = 40;
  size_t opt_make_span = 0UL;
  size_t N=8UL;
  size_t shift_x=1, shift_y=1;

  double *init_solution = NULL;
  size_t var_count;
  {
    // Create memory for variables //
    strip_packing_lp_t strip_packing_lp;
    {
      strip_packing_lp.reset(input, input+N, resource_bound, makespan_bound);
      var_count = strip_packing_lp.total_variable_count();
      init_solution = new double[var_count+1UL]; // one indexing //
      ASSERT_TRUE(init_solution != NULL);
    }

    // first solve the solution optimally //
    glpk_solver_t solver;
    solution_iterator_t sol_begin = solver.solution_begin(input, input+N,
          resource_bound/2UL, makespan_bound, false, NULL, NULL,
          init_solution, init_solution+var_count+1),
                        sol_end = solver.solution_end(); 
    size_t output_count=0;

    while (sol_begin != sol_end) {
      ASSERT_TRUE(output_count <= N); 
      placement_t placement = *sol_begin;
      input[output_count].place(placement.x_+shift_x, placement.y_+shift_y);
      ++output_count;
      ++sol_begin;
    }
    ASSERT_EQ(output_count, N);


    // Check non-overlapping //
    for (size_t i=0; i<N; i++) {
      for (size_t j=i+1UL; j<N; j++) {
        EXPECT_FALSE(mv::Overlap_Util::overlapping_rectangles_without_touching(
                input[i], input[j]));
      }
    }

    { 
      // shift the solution //
      variable_iterator_t vitr = strip_packing_lp.variable_iterator_begin();
      variable_iterator_t vitr_end = strip_packing_lp.variable_iterator_end();
      size_t col_id = 1;
      for (;vitr!=vitr_end; ++vitr,++col_id) {
        std::string var_name(*vitr);
        char const * const str = var_name.c_str();
        if (str[0] == 'x') {
          init_solution[col_id] += double(shift_x);
        } else if (str[0] == 'y') {
          init_solution[col_id] += double(shift_y);
        } else if (str[0] == 'z') {
          init_solution[col_id] += double(shift_y);
        }
      }
    }

    {
      Packing_Visualizer packing_visualizer("shift_packing_narrow_bin.tex");
      packing_visualizer.generate(resource_bound, makespan_bound, input,
            input+N);
    }
  } 


  { 
    // Run solver with init_solution as starting point //
    glpk_solver_t solver;
    solution_iterator_t sol_begin = solver.solution_begin(input, input+N,
          resource_bound, makespan_bound, false,
          init_solution, init_solution+var_count+1),
                        sol_end = solver.solution_end(); 
    size_t output_count=0;

    while (sol_begin != sol_end) {
      ASSERT_TRUE(output_count <= N); 
      placement_t placement = *sol_begin;
      input[output_count].place(placement.x_, placement.y_);
      ++output_count;
      ++sol_begin;
    }
    ASSERT_EQ(output_count, N);


    // Check non-overlapping //
    for (size_t i=0; i<N; i++) {
      for (size_t j=i+1UL; j<N; j++) {
        EXPECT_FALSE(mv::Overlap_Util::overlapping_rectangles_without_touching(
                input[i], input[j]));
      }
    }
  }

  { 
    // Run default solver //
    glpk_solver_t solver;
    solution_iterator_t sol_begin = solver.solution_begin(input, input+N,
          resource_bound, makespan_bound), sol_end = solver.solution_end(); 
  }

  if (init_solution) {
    delete [] init_solution;
  }


  {
    Packing_Visualizer packing_visualizer("shift_packing_wider_bin.tex");
    packing_visualizer.generate(resource_bound, makespan_bound, input, input+N);
  }
}


TEST_F(Strip_Packing_Fixture, test_opt_with_solution_from_narrow_bin) {
  //TEST PLAN: solve the problem optimally on a narrow bin and use it as
  //a feasible solution to wider bin //
  rect_t input[8] = {
    rect_t(4,10), rect_t(2,20), rect_t(2,5), rect_t(2,5), rect_t(2,4),
    rect_t(2, 3), rect_t(3, 3), rect_t(1,12)
  };

  size_t resource_bound = 8;
  size_t makespan_bound = 40;
  size_t opt_make_span = 0UL;
  size_t N=8UL;

  double *init_solution = NULL;
  size_t var_count;
  {
    // Create memory for variables //
    {
      strip_packing_lp_t strip_packing_lp;
      strip_packing_lp.reset(input, input+N, resource_bound, makespan_bound);
      var_count = strip_packing_lp.total_variable_count();
      init_solution = new double[var_count+1UL]; // one indexing //
      ASSERT_TRUE(init_solution != NULL);
    }

    // first solve the solution optimally //
    glpk_solver_t solver;
    solution_iterator_t sol_begin = solver.solution_begin(input, input+N,
          resource_bound/2UL, makespan_bound, false, NULL, NULL,
          init_solution, init_solution+var_count+1),
                        sol_end = solver.solution_end(); 
    size_t output_count=0;

    while (sol_begin != sol_end) {
      ASSERT_TRUE(output_count <= N); 
      placement_t placement = *sol_begin;
      input[output_count].place(placement.x_, placement.y_);
      ++output_count;
      ++sol_begin;
    }
    ASSERT_EQ(output_count, N);


    // Check non-overlapping //
    for (size_t i=0; i<N; i++) {
      for (size_t j=i+1UL; j<N; j++) {
        EXPECT_FALSE(mv::Overlap_Util::overlapping_rectangles_without_touching(
                input[i], input[j]));
      }
    }

    {
      Packing_Visualizer packing_visualizer("packing_narrow_bin.tex");
      packing_visualizer.generate(resource_bound, makespan_bound, input,
            input+N);
    }
  } 


  { 
    // Run solver with init_solution as starting point //
    glpk_solver_t solver;
    solution_iterator_t sol_begin = solver.solution_begin(input, input+N,
          resource_bound, makespan_bound, false,
          init_solution, init_solution+var_count+1),
                        sol_end = solver.solution_end(); 
    size_t output_count=0;

    while (sol_begin != sol_end) {
      ASSERT_TRUE(output_count <= N); 
      placement_t placement = *sol_begin;
      input[output_count].place(placement.x_, placement.y_);
      ++output_count;
      ++sol_begin;
    }
    ASSERT_EQ(output_count, N);


    // Check non-overlapping //
    for (size_t i=0; i<N; i++) {
      for (size_t j=i+1UL; j<N; j++) {
        EXPECT_FALSE(mv::Overlap_Util::overlapping_rectangles_without_touching(
                input[i], input[j]));
      }
    }
  }

  { 
    // Run default solver //
    glpk_solver_t solver;
    solution_iterator_t sol_begin = solver.solution_begin(input, input+N,
          resource_bound, makespan_bound), sol_end = solver.solution_end(); 
  }

  if (init_solution) {
    delete [] init_solution;
  }


  {
    Packing_Visualizer packing_visualizer("packing_wider_bin.tex");
    packing_visualizer.generate(resource_bound, makespan_bound, input, input+N);
  }
}

TEST_F(Strip_Packing_Fixture, test_opt_with_solution_fat_bins) {
  //TEST PLAN: solve the problem optimally on fat bins and use this as a
  //feasible solution to problem on normal bins//
  rect_t input[8] = {
    rect_t(4,10), rect_t(2,20), rect_t(2,5), rect_t(2,5), rect_t(2,4),
    rect_t(2, 3), rect_t(3, 3), rect_t(1,12)
  };

  size_t resource_bound = 8;
  size_t makespan_bound = 50;
  size_t opt_make_span = 0UL;
  size_t N=8UL;
  size_t fat_delta = 2UL;

  double *init_solution = NULL;
  size_t var_count;
  {
    // STEP-0: grow the rectangles//
    for (size_t i=0; i<N; i++) {
      input[i].width_ += fat_delta;
      input[i].height_ += fat_delta;
    }

    // Create memory for variables //
    {
      strip_packing_lp_t strip_packing_lp;
      strip_packing_lp.reset(input, input+N, resource_bound, makespan_bound);
      var_count = strip_packing_lp.total_variable_count();
      init_solution = new double[var_count+1UL]; // one indexing //
      ASSERT_TRUE(init_solution != NULL);
    }

    // first solve the solution optimally //
    glpk_solver_t solver;
    solution_iterator_t sol_begin = solver.solution_begin(input, input+N,
          resource_bound, makespan_bound, false, NULL, NULL,
          init_solution, init_solution+var_count+1),
                        sol_end = solver.solution_end(); 
    size_t output_count=0;

    while (sol_begin != sol_end) {
      ASSERT_TRUE(output_count <= N); 
      placement_t placement = *sol_begin;
      input[output_count].place(placement.x_, placement.y_);
      ++output_count;
      ++sol_begin;
    }
    ASSERT_EQ(output_count, N);


    // Check non-overlapping //
    for (size_t i=0; i<N; i++) {
      for (size_t j=i+1UL; j<N; j++) {
        EXPECT_FALSE(mv::Overlap_Util::overlapping_rectangles_without_touching(
                input[i], input[j]));
      }
    }


    // shrink the rectangles//
    for (size_t i=0; i<N; i++) {
      input[i].width_ -= fat_delta;
      input[i].height_ -= fat_delta;
    }

    {
      Packing_Visualizer packing_visualizer("packing_fat_bin.tex");
      packing_visualizer.generate(resource_bound, makespan_bound, input,
            input+N);
    }
  } 


  { 
    // Run solver with init_solution as starting point //
    glpk_solver_t solver;
    solution_iterator_t sol_begin = solver.solution_begin(input, input+N,
          resource_bound, makespan_bound, false,
          init_solution, init_solution+var_count+1),
                        sol_end = solver.solution_end(); 
    size_t output_count=0;

    while (sol_begin != sol_end) {
      ASSERT_TRUE(output_count <= N); 
      placement_t placement = *sol_begin;
      input[output_count].place(placement.x_, placement.y_);
      ++output_count;
      ++sol_begin;
    }
    ASSERT_EQ(output_count, N);

    // Check non-overlapping //
    for (size_t i=0; i<N; i++) {
      for (size_t j=i+1UL; j<N; j++) {
        EXPECT_FALSE(mv::Overlap_Util::overlapping_rectangles_without_touching(
                input[i], input[j]));
      }
    }
  }

  { 
    // Run default solver //
    glpk_solver_t solver;
    solution_iterator_t sol_begin = solver.solution_begin(input, input+N,
          resource_bound, makespan_bound), sol_end = solver.solution_end(); 
  }

  if (init_solution) {
    delete [] init_solution;
  }

  {
    Packing_Visualizer packing_visualizer("packing_opt_with_fat_input_bin.tex");
    packing_visualizer.generate(resource_bound, makespan_bound, input, input+N);
  }
}

#if 0
TEST_F(Strip_Packing_Fixture, test_packing_with_slightly_changed_opt_solution) {
  //TEST PLAN: solve the problem optimally and slightly change optimal solution
  //move B_1 and B_3 up by 5 units.
  rect_t input[8] = {
    rect_t(4,10), rect_t(2,20), rect_t(2,5), rect_t(2,5), rect_t(2,4),
    rect_t(2, 3), rect_t(3, 3), rect_t(1,12)
  };

  size_t resource_bound = 8;
  size_t makespan_bound = 30;
  size_t opt_make_span = 0UL;
  size_t N=8UL;

  double *init_solution = NULL;
  size_t var_count;
  {
    // Create memory for variables //
    size_t y_1_id, y_3_id, z_h_id;
    {
      strip_packing_lp_t strip_packing_lp;
      strip_packing_lp.reset(input, input+N, resource_bound, makespan_bound);
      var_count = strip_packing_lp.total_variable_count();
      init_solution = new double[var_count+1UL]; // one indexing //
      ASSERT_TRUE(init_solution != NULL);

      variable_iterator_t vitr = strip_packing_lp.variable_iterator_begin();
      variable_iterator_t vitr_end = strip_packing_lp.variable_iterator_end();
      size_t col_id=1;
      for (;vitr != vitr_end; ++vitr) {
        std::string variable_name(*vitr);
        if (variable_name == "y_3") {
          y_3_id = col_id;
        } else if (variable_name == "y_1") {
          y_1_id = col_id;
        } else if (variable_name == "z_H") {
          z_h_id = col_id;
        }
        ++col_id;
      }

      ASSERT_TRUE(y_3_id <= var_count+1);
      ASSERT_TRUE(y_1_id <= var_count+1);
      ASSERT_TRUE(y_1_id < y_3_id);
      ASSERT_EQ(z_h_id, var_count);
    }

    // first solve the solution optimally //
    glpk_solver_t solver;
    solution_iterator_t sol_begin = solver.solution_begin(input, input+N,
          resource_bound, makespan_bound, false, NULL, NULL,
          init_solution, init_solution+var_count+1),
                        sol_end = solver.solution_end(); 
    size_t output_count=0;

    while (sol_begin != sol_end) {
      ASSERT_TRUE(output_count <= N); 
      placement_t placement = *sol_begin;

      if ((output_count==0) || (output_count==2)) {
        placement.y_ += 5; 
      }

      input[output_count].place(placement.x_, placement.y_);
      ++output_count;
      ++sol_begin;
    }
    ASSERT_EQ(output_count, N);

    // preturb final solution //
    init_solution[y_3_id] += double(5.0);
    init_solution[y_1_id] += double(5.0);
    init_solution[z_h_id] += double(5.0);

    {
      Packing_Visualizer packing_visualizer("preturbed_solution.tex");
      packing_visualizer.generate(resource_bound, makespan_bound, input,
            input+N);
    }
  } 


  { 
    // Run solver with init_solution as starting point //
    glpk_solver_t solver;
    solution_iterator_t sol_begin = solver.solution_begin(input, input+N,
          resource_bound, makespan_bound, false,
          init_solution, init_solution+var_count+1),
                        sol_end = solver.solution_end(); 
    size_t output_count=0;

    while (sol_begin != sol_end) {
      ASSERT_TRUE(output_count <= N); 
      placement_t placement = *sol_begin;
      input[output_count].place(placement.x_, placement.y_);
      ++output_count;
      ++sol_begin;
    }
    ASSERT_EQ(output_count, N);
  }

  if (init_solution) {
    delete [] init_solution;
  }


  {
    Packing_Visualizer packing_visualizer("preturbed_final_solution.tex");
    packing_visualizer.generate(resource_bound, makespan_bound, input, input+N);
  }
}
#endif



TEST(Strip_Packing_Linear_Program, derive_direction_variables) {
  rect_t a(0, 0, 10, 10), b(20, 20, 10, 10);
  rect_t::unit_t t_R=1, t_L=1, t_B=1, t_U=1;

  strip_packing_lp_t::derive_direction_variables_from_placement_pair(a, b,
        t_R, t_L, t_U, t_B);

  ASSERT_TRUE( (t_R + t_L + t_U + t_B) <= 3 );
}


TEST(Strip_Packing_Linear_Program, derive_direction_variables_overlap) {
  rect_t a(0, 0, 10, 10), b(5, 5, 10, 10);
  rect_t::unit_t t_R=1, t_L=1, t_B=1, t_U=1;

  strip_packing_lp_t::derive_direction_variables_from_placement_pair(a, b,
        t_R, t_L, t_U, t_B);

  ASSERT_FALSE( (t_R + t_L + t_U + t_B) <= 3 );
}

//NOTE: touch is OK //
TEST(Strip_Packing_Linear_Program,
      derive_direction_variables_overlap_degenerate_point_rect_touch) {
  rect_t a(5, 5, 0, 0), b(5, 5, 10, 10);
  rect_t::unit_t t_R=1, t_L=1, t_B=1, t_U=1;

  strip_packing_lp_t::derive_direction_variables_from_placement_pair(a, b,
        t_R, t_L, t_U, t_B);

  ASSERT_TRUE( (t_R + t_L + t_U + t_B) <= 3 );
}

TEST(Strip_Packing_Linear_Program,
      derive_direction_variables_overlap_degenerate_point_rect_inside) {
  rect_t a(6, 6, 0, 0), b(5, 5, 10, 10);
  rect_t::unit_t t_R=1, t_L=1, t_B=1, t_U=1;

  strip_packing_lp_t::derive_direction_variables_from_placement_pair(a, b,
        t_R, t_L, t_U, t_B);

  ASSERT_FALSE( (t_R + t_L + t_U + t_B) <= 3 );
}

TEST(Strip_Packing_Linear_Program,
      derive_direction_variables_overlap_degenerate_point_rect_outside) {
  rect_t a(50, 50, 0, 0), b(5, 5, 10, 10);
  rect_t::unit_t t_R=1, t_L=1, t_B=1, t_U=1;

  strip_packing_lp_t::derive_direction_variables_from_placement_pair(a, b,
        t_R, t_L, t_U, t_B);

  ASSERT_TRUE( (t_R + t_L + t_U + t_B) <= 3 );
}

TEST(Strip_Packing_Linear_Program,
      derive_direction_variables_overlap_sliding_rectangles) {
  rect_t a(6, 5, 2, 1), b(5, 5, 10, 1);
  rect_t::unit_t t_R=1, t_L=1, t_B=1, t_U=1;

  strip_packing_lp_t::derive_direction_variables_from_placement_pair(a, b,
        t_R, t_L, t_U, t_B);

  ASSERT_FALSE( (t_R + t_L + t_U + t_B) <= 3 );
}

TEST(Strip_Packing_Linear_Program,
      derive_direction_variables_overlap_degenerate_segment_rect_inside) {
  rect_t a(6, 6, 1, 0), b(5, 5, 10, 10);
  rect_t::unit_t t_R=1, t_L=1, t_B=1, t_U=1;

  strip_packing_lp_t::derive_direction_variables_from_placement_pair(a, b,
        t_R, t_L, t_U, t_B);

  ASSERT_FALSE( (t_R + t_L + t_U + t_B) <= 3 );
}


TEST(Strip_Packing_Linear_Program, derive_gap_variables_empty_test) {

  dag_t input_dag;
  strip_packing_lp_t strip_packing_lp;
  rect_t placed_rects;
  std::vector<size_t> gap_variables;

  strip_packing_lp_t::derive_gap_variables_from_packing(
      input_dag, &placed_rects, &placed_rects,
      std::back_inserter(gap_variables) );

  EXPECT_TRUE(gap_variables.empty());
}

TEST_F(Strip_Packing_Fixture, linear_dag_solve_and_verify_derivation) {
  //
  // Test Plan: 
  //  1. Solve the problem with the LpSolve and collect all variables.
  //  2. Extract only placement variables and derive the remaining variables
  //     and verify both variables are exactly same.

  typename dag_t::adjacency_map_t edges = {{"A", {"B"}}, {"B", {"C"}},
      {"C", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights = {{"A", 10}, {"B", 20}, {"C", 5}};
  typename dag_t::resource_cost_model_t widths = {{"A", 4}, {"B", 6}, {"C", 5}};

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 15;
  size_t makespan_bound = 40;

  double *final_solution = NULL;
  // Allocate memory for the solution variables //
  strip_packing_lp_t strip_packing_lp;
  strip_packing_lp.reset_from_dag(input_dag_, resource_bound, makespan_bound);
  
  size_t var_count = strip_packing_lp.total_variable_count();
  final_solution = new double[var_count+1UL]; // one indexing //
  double *final_solution_end = final_solution + var_count + 1UL;
  ASSERT_TRUE(final_solution != NULL);


  glpk_solver_t solver;
  solution_iterator_t sol_beg =
    solver.solve_resource_constraint_scheduling_problem_and_get_final_solution(
        input_dag_, resource_bound, makespan_bound, final_solution,
        final_solution_end);
  solution_iterator_t sol_end = solver.solution_end();

  ASSERT_FALSE(sol_beg == sol_end);
  
  std::vector<rect_t> placements;

  for (const_operation_iterator_t nitr=input_dag_.begin();
            nitr != input_dag_.end(); ++nitr) {
    placements.push_back( rect_t(dag_t::width(input_dag_, *nitr),
          dag_t::height(input_dag_, *nitr) ) );
  }

  size_t placement_idx = 0UL;
  for (;sol_beg != sol_end; ++sol_beg, ++placement_idx) {
    placement_t placement = *sol_beg;
    placements[placement_idx].place(placement.x_, placement.y_);
    placements[placement_idx].print();
  }


  // Now verify the gap variables //
  std::vector<unit_t> derived_gap_variables;
  strip_packing_lp_t::derive_gap_variables_from_packing(input_dag_,
      placements.begin(), placements.end(),
      std::back_inserter(derived_gap_variables) );

  ASSERT_EQ(derived_gap_variables.size(), strip_packing_lp.dimension());
  for (size_t i=0; i<derived_gap_variables.size(); i++) {
    placements[i].print();
    ASSERT_EQ( derived_gap_variables[i],
          unit_t( final_solution[
                    strip_packing_lp.get_gap_variable_col_id(i+1) ] ) );
  }

  // Add gap to each placement so that the direction variables come out correct
  for (size_t i=0; i<derived_gap_variables.size(); i++) {
    placements[i].height_ +=
      unit_t( final_solution[ strip_packing_lp.get_gap_variable_col_id(i+1)] );
  }


  // Now verify the direction variables //
  for (size_t i=1; i<=placements.size(); i++) {
    for (size_t j=i+1; j<=placements.size(); j++) {
      
      size_t dir_col_index[4UL] = {
            strip_packing_lp.get_col_id_RIGHT(i, j),
            strip_packing_lp.get_col_id_LEFT(i, j),
            strip_packing_lp.get_col_id_UP(i, j),
            strip_packing_lp.get_col_id_BOTTOM(i, j)
          };

      unit_t derived_vars[4];

      strip_packing_lp_t::derive_direction_variables_from_placement_pair(
          placements[i-1], placements[j-1], derived_vars[0], derived_vars[1],
          derived_vars[2], derived_vars[3], true);

      printf(" direction_variables = {%d %d %d %d}\n", derived_vars[0], 
          derived_vars[1], derived_vars[2], derived_vars[3]);
      printf(" gold_variables = {%d %d %d %d}\n", 
          unit_t( final_solution[ dir_col_index[0] ] ), 
          unit_t( final_solution[ dir_col_index[1] ] ), 
          unit_t( final_solution[ dir_col_index[2] ] ), 
          unit_t( final_solution[ dir_col_index[3] ] )
      );

      for (size_t i=0; i<4; i++) {
        ASSERT_EQ( derived_vars[i],
              unit_t(final_solution[ dir_col_index[i] ]) );
      }
    }
  }



  if (final_solution) {
    delete final_solution;
  }
}


//TODO(vamsikku) : remove duplication of the test between previous and current
//the only parameter is the DAG.
TEST_F(Strip_Packing_Fixture, diamond_dag_solve_and_verify_derivation) {
  //
  // Test Plan: 
  //  1. Solve the problem with the LpSolve and collect all variables.
  //  2. Extract only placement variables and derive the remaining variables
  //     and verify both variables are exactly same.

  typename dag_t::adjacency_map_t edges =
    { {"A", {"B", "C"}}, {"B", {"D"}}, {"C", {"D"}}, {"D", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}, {"D", 5} };
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 6}, {"C", 2}, {"D", 2} };


  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 15;
  size_t makespan_bound = 40;

  double *final_solution = NULL;
  // Allocate memory for the solution variables //
  strip_packing_lp_t strip_packing_lp;
  strip_packing_lp.reset_from_dag(input_dag_, resource_bound, makespan_bound);
  
  size_t var_count = strip_packing_lp.total_variable_count();
  final_solution = new double[var_count+1UL]; // one indexing //
  double *final_solution_end = final_solution + var_count + 1UL;
  ASSERT_TRUE(final_solution != NULL);


  glpk_solver_t solver;
  solution_iterator_t sol_beg =
    solver.solve_resource_constraint_scheduling_problem_and_get_final_solution(
        input_dag_, resource_bound, makespan_bound, final_solution,
        final_solution_end);
  solution_iterator_t sol_end = solver.solution_end();

  ASSERT_FALSE(sol_beg == sol_end);
  
  std::vector<rect_t> placements;

  for (const_operation_iterator_t nitr=input_dag_.begin();
            nitr != input_dag_.end(); ++nitr) {
    placements.push_back( rect_t(dag_t::width(input_dag_, *nitr),
          dag_t::height(input_dag_, *nitr) ) );
  }

  size_t placement_idx = 0UL;
  for (;sol_beg != sol_end; ++sol_beg, ++placement_idx) {
    placement_t placement = *sol_beg;
    placements[placement_idx].place(placement.x_, placement.y_);
    placements[placement_idx].print();
  }


  // Now verify the gap variables //
  std::vector<unit_t> derived_gap_variables;
  strip_packing_lp_t::derive_gap_variables_from_packing(input_dag_,
      placements.begin(), placements.end(),
        std::back_inserter(derived_gap_variables) );

  ASSERT_EQ(derived_gap_variables.size(), strip_packing_lp.dimension());
  for (size_t i=0; i<derived_gap_variables.size(); i++) {
    placements[i].print();
    ASSERT_EQ( derived_gap_variables[i],
          unit_t( final_solution[
                    strip_packing_lp.get_gap_variable_col_id(i+1) ] ) );
  }

  // Add gap to each placement so that the direction variables come out correct
  for (size_t i=0; i<derived_gap_variables.size(); i++) {
    placements[i].height_ +=
      unit_t( final_solution[ strip_packing_lp.get_gap_variable_col_id(i+1)] );
  }


  // Now verify the direction variables //
  for (size_t i=1; i<=placements.size(); i++) {
    for (size_t j=i+1; j<=placements.size(); j++) {
      
      size_t dir_col_index[4UL] = {
            strip_packing_lp.get_col_id_RIGHT(i, j),
            strip_packing_lp.get_col_id_LEFT(i, j),
            strip_packing_lp.get_col_id_UP(i, j),
            strip_packing_lp.get_col_id_BOTTOM(i, j)
          };

      unit_t derived_vars[4];

      strip_packing_lp_t::derive_direction_variables_from_placement_pair(
          placements[i-1], placements[j-1], derived_vars[0], derived_vars[1],
          derived_vars[2], derived_vars[3], true);

      printf(" direction_variables = {%d %d %d %d}\n", derived_vars[0], 
          derived_vars[1], derived_vars[2], derived_vars[3]);
      printf(" gold_variables = {%d %d %d %d}\n", 
          unit_t( final_solution[ dir_col_index[0] ] ), 
          unit_t( final_solution[ dir_col_index[1] ] ), 
          unit_t( final_solution[ dir_col_index[2] ] ), 
          unit_t( final_solution[ dir_col_index[3] ] )
      );

      for (size_t i=0; i<4; i++) {
        ASSERT_EQ( derived_vars[i],
              unit_t(final_solution[ dir_col_index[i] ]) );
      }
    }
  }



  if (final_solution) {
    delete final_solution;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Following tests generate a packing from the list scheduler (production) and
// takes the solution and calls the LpSolver
////////////////////////////////////////////////////////////////////////////////
TEST_F(Strip_Packing_Fixture, generate_packing_from_scheduler) {
  typename dag_t::adjacency_map_t edges =
    { {"A", {"B", "C"}}, {"B", {"D"}}, {"C", {"D"}}, {"D", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}, {"D", 5} };
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 6}, {"C", 2}, {"D", 2} };

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 12;
  size_t makespan_bound = 45;

  // Generate packing from feasible scheduler //
  std::vector<rect_t> feasible_scheduler_placements;
  bool has_no_spills =
      strip_packing_lp_t::generate_packing_from_feasible_scheduler(input_dag_,
          resource_bound, std::back_inserter(feasible_scheduler_placements) );

  {
    // visualize the feasible_scheduler_placements //
    Packing_Visualizer packing_visualizer("feasible_solution.tex");
    packing_visualizer.generate(input_dag_, resource_bound, makespan_bound,
        feasible_scheduler_placements.begin(),
        feasible_scheduler_placements.end());
  }


  EXPECT_TRUE(has_no_spills);
  EXPECT_EQ(feasible_scheduler_placements.size(), 4UL);
}

TEST_F(Strip_Packing_Fixture, derive_all_solution_variables) {
  typename dag_t::adjacency_map_t edges =
    { {"A", {"B", "C"}}, {"B", {"D"}}, {"C", {"D"}}, {"D", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}, {"D", 5} };
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 6}, {"C", 2}, {"D", 2} };

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 12;
  size_t makespan_bound = 45;
  std::vector<int> solution_from_scheduler;
  solution_from_scheduler.push_back(0);

  strip_packing_lp_t::derive_all_solution_variables_from_feasible_scheduler(
      input_dag_, resource_bound, std::back_inserter(solution_from_scheduler));

  EXPECT_FALSE( solution_from_scheduler.empty() );
  EXPECT_EQ( solution_from_scheduler.back(), 35);

  // also check if the size of solution_from_scheduler has same variables //
  {
    strip_packing_lp_t strip_packing_lp;
    strip_packing_lp.reset_from_dag(input_dag_, resource_bound, makespan_bound);
    EXPECT_EQ(solution_from_scheduler.size(),
          strip_packing_lp.total_variable_count()+1UL);
  }

}

TEST_F(Strip_Packing_Fixture, generate_packing_from_scheduler_optimal) {
  typename dag_t::adjacency_map_t edges =
    { {"A", {"B", "C"}}, {"B", {"D"}}, {"C", {"D"}}, {"D", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}, {"D", 5} };
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 6}, {"C", 2}, {"D", 2} };

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 12;
  size_t makespan_bound = 45;

  solve_and_verify(resource_bound, makespan_bound, true, "optimal_diamond.tex",
      false, false);
}


TEST_F(Strip_Packing_Fixture, improve_scheduler_feasible_solution) {
  typename dag_t::adjacency_map_t edges =
    { {"A", {"B", "C"}}, {"B", {"D"}}, {"C", {"D"}}, {"D", {}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}, {"D", 5} };
  typename dag_t::resource_cost_model_t widths =
    { {"A", 4}, {"B", 6}, {"C", 2}, {"D", 2} };

  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);

  size_t resource_bound = 12;
  size_t makespan_bound = 45;

  EXPECT_TRUE(solve_and_verify_iteratively_improved_solution(resource_bound,
        makespan_bound, "scheduler_orig_sol_test0.tex",
          "improved_orig_sol_test0.tex") );
}

TEST_F(Strip_Packing_Fixture,
      improve_scheduler_feasible_solution_prefetch_test) {

  typename dag_t::adjacency_map_t edges = { {"A", {"B"}}, {"B", {"C"}},
    {"C", {"D"}}, {"D", {}}, {"$W_A$", {"A"}}, {"$W_B$", {"B"}},
    {"$W_C$", {"C"}}, {"$W_D$", {"D"}} };
  // delays = heights //
  typename dag_t::delay_cost_model_t heights =
    { {"A", 10}, {"B", 20}, {"C", 5}, {"D", 5},
      {"$W_A$", 4}, {"$W_B$", 3}, {"$W_C$", 3}, {"$W_D$", 12}};

  // No-prefetch for C //
  typename dag_t::resource_cost_model_t widths =
    { {"$W_A$", 2}, {"A", 4}, {"$W_B$", 2}, {"B", 2}, 
      {"$W_C$", 3}, {"C", 2}, {"$W_D$", 1}, {"D", 2}  };

  typename dag_t::data_op_set_t data_ops = {"$W_A$", "$W_B", "$W_C", "$W_D"};
  // Create DAG with dependencies and delays of each of these tasks //
  reset_input(edges, heights, widths);
  input_dag_.reset_data_op_set(data_ops);

  size_t resource_bound = 11;
  size_t makespan_bound = 55;

  EXPECT_TRUE(solve_and_verify_iteratively_improved_solution(resource_bound,
        makespan_bound, "scheduler_orig_sol_test1.tex",
          "improved_orig_sol_test1.tex") );
}


TEST_F(Strip_Packing_Fixture, Rebuild_Data_Flow_spilled_test) {
  size_t resource_bound = 10;
  size_t makespan_bound = 55;

  // load test input //
  load_optimal_prefetch_test_input();

  dag_editor_t dag_editor(input_dag_);
  schedule_order_t schedule;

  // get schedule from the production scheduler //
  get_feasible_schedule(schedule, resource_bound);

  size_t result = mv::Scheduler_DAG_Util::rebuild_data_flow_dag_from_schedule
      <typename schedule_order_t::const_iterator, sched_traits_t>(dag_editor,
          schedule.begin(), schedule.end());

  dag_editor.print();
  EXPECT_EQ(dag_editor.added_edges_.size(), 6UL);
  EXPECT_EQ(dag_editor.removed_edges_.size(), 2UL);
  EXPECT_EQ(dag_editor.added_nodes_.size(), 4UL);
}

TEST_F(Strip_Packing_Fixture, Rebuild_Data_Flow_Apply_Edits) {
  size_t resource_bound = 10;
  size_t makespan_bound = 55;

  // load test input //
  load_optimal_prefetch_test_input();

  dag_editor_t dag_editor(input_dag_);
  schedule_order_t schedule;
  // get schedule from the production scheduler //
  get_feasible_schedule(schedule, resource_bound);

  size_t result = mv::Scheduler_DAG_Util::rebuild_data_flow_dag_from_schedule
      <typename schedule_order_t::const_iterator, sched_traits_t>(dag_editor,
          schedule.begin(), schedule.end());

  std::vector< std::pair<operation_t, operation_t> >
      expected_spill_data_flow_edges= {
        {"$W_B$", "$W_B$spWr-0"}, {"$W_B$spWr-0", "$W_B$spRd-0"},
          {"$W_B$spRd-0", "B"},
        {"$W_C$", "$W_C$spWr-0"}, {"$W_C$spWr-0", "$W_C$spRd-0"},
          {"$W_C$spRd-0", "C"},
      };


  // before edits none of these edges should exist //
  for ( auto e : expected_spill_data_flow_edges) {
    ASSERT_FALSE(input_dag_.is_valid_edge(e.first, e.second));
  }


  sched_traits_t::apply_edits(dag_editor, input_dag_);

  // after edits we should see new data flows.//
  for ( auto e : expected_spill_data_flow_edges) {
    ASSERT_TRUE(input_dag_.is_valid_edge(e.first, e.second));
  }

  ASSERT_FALSE(input_dag_.is_valid_edge("$W_B", "B"));
  ASSERT_FALSE(input_dag_.is_valid_edge("$W_C", "C"));
}


TEST_F(Strip_Packing_Fixture, Rebuild_Data_Flow_Updated_Schedule) {
  size_t resource_bound = 10;
  size_t makespan_bound = 55;

  // load test input //
  load_optimal_prefetch_test_input();

  dag_editor_t dag_editor(input_dag_);
  schedule_order_t original_schedule, updated_schedule;

  // get schedule from the production scheduler //
  get_feasible_schedule(original_schedule, resource_bound);

  size_t result = mv::Scheduler_DAG_Util::rebuild_data_flow_dag_from_schedule
      <typename schedule_order_t::const_iterator,
        sched_traits_t, scheduled_op_info_t>(dag_editor,
            original_schedule.begin(), original_schedule.end(),
              std::back_inserter(updated_schedule) );

  EXPECT_TRUE(result > 0);
  EXPECT_TRUE(updated_schedule.size() == original_schedule.size());

  for (size_t i=0; i<updated_schedule.size(); i++) {
    ASSERT_EQ( updated_schedule[i].begin_resource(),
          original_schedule[i].begin_resource());
    ASSERT_EQ( updated_schedule[i].end_resource(),
          original_schedule[i].end_resource());
    ASSERT_EQ(updated_schedule[i].time_, original_schedule[i].time_);

    if (!strcmp( original_schedule[i].op_type_name(), "ORIGINAL")) {
      ASSERT_EQ(updated_schedule[i], original_schedule[i]);
    } else if (!strcmp(original_schedule[i].op_type_name(), "SPILLED_WRITE")) {
      ASSERT_TRUE(!strcmp(updated_schedule[i].op_type_name(), "ORIGINAL"));
      ASSERT_TRUE( strstr(updated_schedule[i].op_.c_str(), "spWr-") != NULL);
    }  else if (!strcmp(original_schedule[i].op_type_name(), "SPILLED_READ")) {
      ASSERT_TRUE(!strcmp(updated_schedule[i].op_type_name(), "ORIGINAL"));
      ASSERT_TRUE( strstr(updated_schedule[i].op_.c_str(), "spRd-") != NULL);
    } else {
      ASSERT_TRUE(false);
    }
  }
}

TEST_F(Strip_Packing_Fixture, Generate_Packing_From_Spilled_Schedule) {
  size_t resource_bound = 9;
  size_t makespan_bound = 55;

  // load test input //
  load_optimal_prefetch_test_input();

  std::vector<rect_t> spilled_feasible_scheduler_placements;

  bool has_no_spills =
      strip_packing_lp_t::generate_packing_from_feasible_scheduler(
          input_dag_, resource_bound,
          std::back_inserter(spilled_feasible_scheduler_placements) );

  update_makespan_bound(makespan_bound,
      spilled_feasible_scheduler_placements.begin(),
      spilled_feasible_scheduler_placements.end());
  {
    // visualize the feasible_scheduler_placements //
    Packing_Visualizer packing_visualizer("spilled_feasible_solution.tex");
    packing_visualizer.generate(input_dag_, resource_bound, makespan_bound,
        spilled_feasible_scheduler_placements.begin(),
        spilled_feasible_scheduler_placements.end());
  }

  EXPECT_FALSE(has_no_spills);
}

TEST_F(Strip_Packing_Fixture, Derive_Variables_From_Spilled_Solution) {
  size_t resource_bound = 9;
  size_t makespan_bound = 71;

  // load test input //
  load_optimal_prefetch_test_input();

  std::vector<double> solution_from_scheduler;
  solution_from_scheduler.push_back(0);
  
  strip_packing_lp_t::derive_all_solution_variables_from_feasible_scheduler(
      input_dag_, resource_bound, std::back_inserter(solution_from_scheduler) );
  
  std::pair<bool, size_t> start_solution_result =
    verify_solution(resource_bound, makespan_bound,
        solution_from_scheduler.data(),
        solution_from_scheduler.data()+solution_from_scheduler.size(),
        "spilled_variable_derivation.tex");
  
  EXPECT_TRUE(start_solution_result.first);
  EXPECT_EQ(start_solution_result.second, 71); 
}

TEST_F(Strip_Packing_Fixture, Improve_Spilled_Solution) {
  size_t resource_bound = 9;
  size_t makespan_bound = 71;

  // load test input //
  load_optimal_prefetch_test_input();

  EXPECT_TRUE(solve_and_verify_iteratively_improved_solution(resource_bound,
        makespan_bound, "spilled_scheduler_orig_sol_test0.tex",
          "spilled_improved_orig_sol_test0.tex") );
}

