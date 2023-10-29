/*################################################################################
  ##
  ##   Copyright (C) 2016-2022 Keith O'Hara
  ##
  ##   This file is part of the GCE-Math C++ library.
  ##
  ##   Licensed under the Apache License, Version 2.0 (the "License");
  ##   you may not use this file except in compliance with the License.
  ##   You may obtain a copy of the License at
  ##
  ##       http://www.apache.org/licenses/LICENSE-2.0
  ##
  ##   Unless required by applicable law or agreed to in writing, software
  ##   distributed under the License is distributed on an "AS IS" BASIS,
  ##   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  ##   See the License for the specific language governing permissions and
  ##   limitations under the License.
  ##
  ################################################################################*/

/*
 * Gauss-Legendre quadrature: 50 points
 */

static const long double gauss_legendre_50_points[50] = {
    -0.03109833832718887611232898966595L,
    0.03109833832718887611232898966595L,
    -0.09317470156008614085445037763960L,
    0.09317470156008614085445037763960L,
    -0.15489058999814590207162862094111L,
    0.15489058999814590207162862094111L,
    -0.21600723687604175684728453261710L,
    0.21600723687604175684728453261710L,
    -0.27628819377953199032764527852113L,
    0.27628819377953199032764527852113L,
    -0.33550024541943735683698825729107L,
    0.33550024541943735683698825729107L,
    -0.39341431189756512739422925382382L,
    0.39341431189756512739422925382382L,
    -0.44980633497403878914713146777838L,
    0.44980633497403878914713146777838L,
    -0.50445814490746420165145913184914L,
    0.50445814490746420165145913184914L,
    -0.55715830451465005431552290962580L,
    0.55715830451465005431552290962580L,
    -0.60770292718495023918038179639183L,
    0.60770292718495023918038179639183L,
    -0.65589646568543936078162486400368L,
    0.65589646568543936078162486400368L,
    -0.70155246870682225108954625788366L,
    0.70155246870682225108954625788366L,
    -0.74449430222606853826053625268219L,
    0.74449430222606853826053625268219L,
    -0.78455583290039926390530519634099L,
    0.78455583290039926390530519634099L,
    -0.82158207085933594835625411087394L,
    0.82158207085933594835625411087394L,
    -0.85542976942994608461136264393476L,
    0.85542976942994608461136264393476L,
    -0.88596797952361304863754098246675L,
    0.88596797952361304863754098246675L,
    -0.91307855665579189308973564277166L,
    0.91307855665579189308973564277166L,
    -0.93665661894487793378087494727250L,
    0.93665661894487793378087494727250L,
    -0.95661095524280794299774564415662L,
    0.95661095524280794299774564415662L,
    -0.97286438510669207371334410460625L,
    0.97286438510669207371334410460625L,
    -0.98535408404800588230900962563249L,
    0.98535408404800588230900962563249L,
    -0.99403196943209071258510820042069L,
    0.99403196943209071258510820042069L,
    -0.99886640442007105018545944497422L,
    0.99886640442007105018545944497422L
};

static const long double gauss_legendre_50_weights[50] = {
    0.06217661665534726232103310736061L,
    0.06217661665534726232103310736061L,
    0.06193606742068324338408750978083L,
    0.06193606742068324338408750978083L,
    0.06145589959031666375640678608392L,
    0.06145589959031666375640678608392L,
    0.06073797084177021603175001538481L,
    0.06073797084177021603175001538481L,
    0.05978505870426545750957640531259L,
    0.05978505870426545750957640531259L,
    0.05860084981322244583512243663085L,
    0.05860084981322244583512243663085L,
    0.05718992564772838372302931506599L,
    0.05718992564772838372302931506599L,
    0.05555774480621251762356742561227L,
    0.05555774480621251762356742561227L,
    0.05371062188899624652345879725566L,
    0.05371062188899624652345879725566L,
    0.05165570306958113848990529584010L,
    0.05165570306958113848990529584010L,
    0.04940093844946631492124358075143L,
    0.04940093844946631492124358075143L,
    0.04695505130394843296563301363499L,
    0.04695505130394843296563301363499L,
    0.04432750433880327549202228683039L,
    0.04432750433880327549202228683039L,
    0.04152846309014769742241197896407L,
    0.04152846309014769742241197896407L,
    0.03856875661258767524477015023639L,
    0.03856875661258767524477015023639L,
    0.03545983561514615416073461100098L,
    0.03545983561514615416073461100098L,
    0.03221372822357801664816582732300L,
    0.03221372822357801664816582732300L,
    0.02884299358053519802990637311323L,
    0.02884299358053519802990637311323L,
    0.02536067357001239044019487838544L,
    0.02536067357001239044019487838544L,
    0.02178024317012479298159206906269L,
    0.02178024317012479298159206906269L,
    0.01811556071348939035125994342235L,
    0.01811556071348939035125994342235L,
    0.01438082276148557441937890892732L,
    0.01438082276148557441937890892732L,
    0.01059054838365096926356968149924L,
    0.01059054838365096926356968149924L,
    0.00675979919574540150277887817799L,
    0.00675979919574540150277887817799L,
    0.00290862255315514095840072434286L,
    0.00290862255315514095840072434286L
};
